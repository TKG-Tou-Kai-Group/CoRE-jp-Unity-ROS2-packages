"""操縦画面を H.264 にして WebSocket で配信する試作。

現状はブラウザが rosbridge (ポート 9090) で JPEG を base64 の JSON として受け取る。
実測 26 KiB/枚 x 10 fps x 1.33 (base64) = 1 系統 2.8 Mbps、8 系統で 23 Mbps。

H.264 なら同じ画をおよそ 1/3 の帯域で送れる。WebRTC でも同じ圧縮率だが、UDP と
シグナリングと NAT 越えが要り、ポート 9090 だけを共有する現行の運用が崩れる。
WebSocket なら 1 ポートのまま送れて、ブラウザ側は WebCodecs で復号できる。

構成:
    ROS の image_compressed (JPEG)
      -> cv2 で復号
      -> ffmpeg (libx264, zerolatency) へ生フレームを流し込む
      -> Annex-B の NAL を読み出す
      -> WebSocket でブラウザへ

WebSocket サーバは外部ライブラリを使わず自前で実装する (コンテナに websockets が
入っていないため)。用途が「1 フレーム = 1 バイナリフレーム」の一方向配信なので、
必要なのはハンドシェイクと送信だけで済む。
"""
import argparse
import base64
import hashlib
import json
import os
import socket
import struct
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


class Encoder:
    """1 系統ぶんの H.264 符号化。ffmpeg を子プロセスとして持つ。"""

    def __init__(self, name, width, height, fps, bitrate, on_packet):
        self.name = name
        self.on_packet = on_packet
        self.proc = subprocess.Popen(
            ['ffmpeg', '-hide_banner', '-loglevel', 'error',
             '-f', 'rawvideo', '-pix_fmt', 'bgr24',
             '-s', f'{width}x{height}', '-r', str(fps), '-i', '-',
             '-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency',
             '-b:v', bitrate, '-maxrate', bitrate, '-bufsize', bitrate,
             # 鍵フレームを短い間隔で入れる。後から参加した視聴者が待たされないため。
             '-g', str(fps * 2), '-keyint_min', str(fps),
             '-bsf:v', 'h264_mp4toannexb' if False else 'null',
             '-f', 'h264', '-'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, bufsize=0)
        self.bytes_out = 0
        self.packets = 0
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        """Annex-B の開始コードで区切って NAL 単位に切り出す。"""
        buf = b''
        while True:
            chunk = self.proc.stdout.read(4096)
            if not chunk:
                return
            buf += chunk
            # 次の開始コードまでをひとまとまりとして送る
            while True:
                idx = buf.find(b'\x00\x00\x00\x01', 4)
                if idx < 0:
                    break
                nal, buf = buf[:idx], buf[idx:]
                if nal:
                    self.bytes_out += len(nal)
                    self.packets += 1
                    self.on_packet(self.name, nal)

    def feed(self, frame_bgr):
        try:
            self.proc.stdin.write(frame_bgr.tobytes())
        except (BrokenPipeError, ValueError):
            pass


class WSServer:
    """一方向配信に必要な部分だけの WebSocket サーバ。"""

    def __init__(self, port):
        self.clients = {}          # conn -> 購読しているストリーム名
        self.lock = threading.Lock()
        self.sock = socket.socket()
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', port))
        self.sock.listen(16)
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            conn, _ = self.sock.accept()
            threading.Thread(target=self._handshake, args=(conn,), daemon=True).start()

    def _handshake(self, conn):
        try:
            data = b''
            while b'\r\n\r\n' not in data:
                b = conn.recv(4096)
                if not b:
                    return
                data += b
            head = data.split(b'\r\n\r\n')[0].decode(errors='replace')
            key = ''
            path = '/'
            for line in head.split('\r\n'):
                if line.lower().startswith('sec-websocket-key:'):
                    key = line.split(':', 1)[1].strip()
                elif line.startswith('GET '):
                    path = line.split(' ')[1]
            accept = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
            conn.sendall(
                ('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n'
                 'Connection: Upgrade\r\n'
                 f'Sec-WebSocket-Accept: {accept}\r\n\r\n').encode())
            stream = path.lstrip('/') or 'default'
            with self.lock:
                self.clients[conn] = stream
        except Exception:
            try:
                conn.close()
            except Exception:
                pass

    def send(self, stream, payload):
        """該当ストリームの購読者へバイナリフレームを送る。"""
        head = bytes([0x82])       # FIN + binary
        n = len(payload)
        if n < 126:
            head += bytes([n])
        elif n < 65536:
            head += bytes([126]) + struct.pack('>H', n)
        else:
            head += bytes([127]) + struct.pack('>Q', n)
        frame = head + payload
        dead = []
        with self.lock:
            targets = [c for c, s in self.clients.items() if s == stream or s == 'default']
        for c in targets:
            try:
                c.sendall(frame)
            except Exception:
                dead.append(c)
        if dead:
            with self.lock:
                for c in dead:
                    self.clients.pop(c, None)
                    try:
                        c.close()
                    except Exception:
                        pass

    def client_count(self):
        with self.lock:
            return len(self.clients)


class Streamer(Node):
    def __init__(self, topic, streams, port, fps, bitrate, raw=False):
        super().__init__('operator_stream')
        self.ws = WSServer(port)
        self.encoders = {}
        self.streams = streams
        self.fps = fps
        self.bitrate = bitrate
        self.jpeg_bytes = 0
        self.jpeg_frames = 0
        self.decode_s = 0.0
        self.raw = raw
        if raw:
            # core_jp_camera_publisher を output_format:=raw|both で動かすと出る。
            # JPEG を復号し直さずに済むので、系統数ぶんの復号コストが丸ごと消える。
            self.create_subscription(Image, topic, self.on_raw, 5)
        else:
            self.create_subscription(CompressedImage, topic, self.on_image, 5)
        self.get_logger().info(
            f"'{topic}' ({'生画像' if raw else 'JPEG'}) を {len(streams)} 系統へ "
            f"H.264 配信 (ws://0.0.0.0:{port}/<name>)")

    def on_raw(self, msg):
        self.jpeg_bytes += len(msg.data)
        self.jpeg_frames += 1
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            frame = frame[:, :, ::-1]
        self._feed(frame, msg.width, msg.height)

    def on_image(self, msg):
        self.jpeg_bytes += len(msg.data)
        self.jpeg_frames += 1
        t0 = time.time()
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        self.decode_s += time.time() - t0
        if frame is None:
            return
        h, w = frame.shape[:2]
        self._feed(frame, w, h)

    def _feed(self, frame, w, h):
        for name in self.streams:
            enc = self.encoders.get(name)
            if enc is None:
                enc = Encoder(name, w, h, self.fps, self.bitrate, self.ws.send)
                self.encoders[name] = enc
            enc.feed(frame)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/sample_robot_1/camera_link/image_compressed')
    ap.add_argument('--streams', type=int, default=8)
    ap.add_argument('--port', type=int, default=9091)
    ap.add_argument('--fps', type=int, default=10)
    ap.add_argument('--bitrate', default='800k')
    ap.add_argument('--secs', type=float, default=40.0)
    ap.add_argument('--raw', action='store_true',
                    help='生画像 (sensor_msgs/Image) を購読する。JPEG 復号を省ける')
    a = ap.parse_args()

    rclpy.init()
    names = [f'robot{i + 1}' for i in range(a.streams)]
    n = Streamer(a.topic, names, a.port, a.fps, a.bitrate, a.raw)
    t0 = time.time()
    while time.time() - t0 < a.secs:
        rclpy.spin_once(n, timeout_sec=0.05)
    dt = time.time() - t0

    enc_bytes = sum(e.bytes_out for e in n.encoders.values())
    enc_pkts = sum(e.packets for e in n.encoders.values())
    jpeg_mbps = n.jpeg_bytes / dt * 8 / 1e6
    h264_mbps = enc_bytes / dt * 8 / 1e6
    print('=== 入力 (現状の操縦画面) ===')
    print('  JPEG %d 枚 / %.1f 秒 = %.1f fps / 1 枚 %.1f KiB / %.2f Mbps (1 系統)'
          % (n.jpeg_frames, dt, n.jpeg_frames / dt,
             n.jpeg_bytes / max(n.jpeg_frames, 1) / 1024, jpeg_mbps))
    print('  JPEG 復号: 合計 %.1f 秒 (1 枚 %.1f ms)'
          % (n.decode_s, 1000 * n.decode_s / max(n.jpeg_frames, 1)))
    print('=== 出力 (H.264) ===')
    print('  %d 系統 / NAL %d 個 / %.2f Mbps (合計) / %.2f Mbps (1 系統)'
          % (len(n.encoders), enc_pkts, h264_mbps, h264_mbps / max(len(n.encoders), 1)))
    print('=== 現状方式との比較 (8 系統に換算) ===')
    print('  rosbridge JSON+base64: %.1f Mbps' % (jpeg_mbps * 1.33 * 8))
    print('  H.264 + WebSocket    : %.1f Mbps' % (h264_mbps / max(len(n.encoders), 1) * 8))
    print('  WS 接続数: %d' % n.ws.client_count())
    print('STREAMDONE')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
