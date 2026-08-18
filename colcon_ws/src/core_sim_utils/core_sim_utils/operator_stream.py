"""操縦画面を H.264 にして WebSocket で配信する。

現状はブラウザが rosbridge (ポート 9090) で JPEG を base64 の JSON として受け取る。
実測 26 KiB/枚 x 10 fps x 1.33 (base64) = 1 系統 2.8 Mbps、8 系統で 23 Mbps。

H.264 なら同じ画をおよそ 1/3 の帯域で送れる。WebRTC でも同じ圧縮率だが、UDP と
シグナリングと NAT 越えが要り、ポート 9090 だけを共有する現行の運用が崩れる。
WebSocket なら 1 ポートのまま送れて、ブラウザ側は WebCodecs で復号できる。

機体ごとに 1 系統。ブラウザは ws://<host>:9091/robot<N> を開く。
tools/robot_<N>_control.html がこの経路で映像を受け取る。

構成 (機体ごと):
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

    def has_client(self, stream):
        """その系統を見ている視聴者が居るか。エンコードの要否判定に使う。"""
        with self.lock:
            return any(s == stream or s == 'default'
                       for s in self.clients.values())


class Streamer(Node):
    """機体ごとに 1 系統を配信する。

    以前は 1 つのトピックを N 系統へ複製するだけの試作で、帯域の比較にしか
    使えなかった。実運用では機体ごとに別の映像が要るので、機体名から
    トピックとストリーム名を組み立てて購読する。

    エンコーダは最初のフレームが来た時点で作る。ロボットは後から生成される
    ので、起動時にはトピックもフレームの大きさも決まっていない。
    """

    def __init__(self, robots, port, fps, bitrate, raw=False,
                 topic_template=None):
        super().__init__('operator_stream')
        self.ws = WSServer(port)
        self.encoders = {}
        self.fps = fps
        self.bitrate = bitrate
        self.raw = raw
        self.jpeg_bytes = 0
        self.jpeg_frames = 0
        self.decode_s = 0.0

        # 生画像は core_jp_camera_publisher が output_format:=raw|both で出す
        # オーバレイ済みのもの。シミュレータが出す素のカメラ画像
        # (/<機体>/camera_link/image_raw) とは別トピックにしてある。
        suffix = 'image_raw_overlay' if raw else 'image_compressed'
        template = topic_template or ('/{robot}/camera_link/' + suffix)
        self.streams = {}
        for i, robot in enumerate(robots):
            topic = template.format(robot=robot)
            name = f'robot{i + 1}'
            self.streams[robot] = name
            if raw:
                self.create_subscription(
                    Image, topic, lambda m, n=name: self.on_raw(n, m), 5)
            else:
                self.create_subscription(
                    CompressedImage, topic,
                    lambda m, n=name: self.on_image(n, m), 5)
        self.get_logger().info(
            f"{len(robots)} 台を H.264 配信 ({'生画像' if raw else 'JPEG'} を入力、"
            f"ws://0.0.0.0:{port}/robot1..robot{len(robots)})")

    def on_raw(self, name, msg):
        self.jpeg_bytes += len(msg.data)
        self.jpeg_frames += 1
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            frame = frame[:, :, ::-1]
        self._feed(name, frame, msg.width, msg.height)

    def on_image(self, name, msg):
        self.jpeg_bytes += len(msg.data)
        self.jpeg_frames += 1
        t0 = time.time()
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        self.decode_s += time.time() - t0
        if frame is None:
            return
        h, w = frame.shape[:2]
        self._feed(name, frame, w, h)

    def _feed(self, name, frame, w, h):
        enc = self.encoders.get(name)
        if enc is None:
            # 視聴者が居ない系統はエンコードしない。8 台ぶんの libx264 を
            # 常時回すと CPU を無駄に食う (カメラ配信だけで既に 8 台で 547%)。
            if not self.ws.has_client(name):
                return
            enc = Encoder(name, w, h, self.fps, self.bitrate, self.ws.send)
            self.encoders[name] = enc
        enc.feed(frame)


def main(args=None):
    """常駐ノードとして動かす。

    以前は --secs で打ち切って帯域の比較値を出すベンチマークだった。
    実運用では止まっては困るので、既定では回り続ける。比較値が要るときは
    --secs を与えると、その時間だけ動いて集計を出す。
    """
    ap = argparse.ArgumentParser()
    ap.add_argument('--robots', nargs='+',
                    default=[f'sample_robot_{i}' for i in range(1, 9)],
                    help='配信する機体。並び順が robot1..robotN に対応する')
    ap.add_argument('--topic-template', default=None,
                    help='購読するトピック。{robot} が機体名に置き換わる')
    ap.add_argument('--port', type=int, default=9091)
    ap.add_argument('--fps', type=int, default=10)
    ap.add_argument('--bitrate', default='800k')
    ap.add_argument('--secs', type=float, default=0.0,
                    help='0 なら常駐。>0 ならその秒数だけ動いて帯域の集計を出す')
    ap.add_argument('--raw', action='store_true',
                    help='生画像 (sensor_msgs/Image) を購読する。JPEG 復号を省ける')
    # ros2 run から渡される --ros-args は argparse が知らないので落とす
    a, _ = ap.parse_known_args()

    rclpy.init(args=args)
    node = Streamer(a.robots, a.port, a.fps, a.bitrate, a.raw, a.topic_template)

    if a.secs <= 0:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    t0 = time.time()
    while time.time() - t0 < a.secs:
        rclpy.spin_once(node, timeout_sec=0.05)
    dt = time.time() - t0

    enc_bytes = sum(e.bytes_out for e in node.encoders.values())
    enc_pkts = sum(e.packets for e in node.encoders.values())
    jpeg_mbps = node.jpeg_bytes / dt * 8 / 1e6
    h264_mbps = enc_bytes / dt * 8 / 1e6
    print('=== 入力 ===')
    print('  %d 枚 / %.1f 秒 = %.1f fps / 1 枚 %.1f KiB / %.2f Mbps'
          % (node.jpeg_frames, dt, node.jpeg_frames / dt,
             node.jpeg_bytes / max(node.jpeg_frames, 1) / 1024, jpeg_mbps))
    if not node.raw:
        print('  JPEG 復号: 合計 %.1f 秒 (1 枚 %.1f ms)'
              % (node.decode_s, node.decode_s / max(node.jpeg_frames, 1) * 1000))
    print('=== 出力 (H.264) ===')
    print('  %d NAL / %.2f Mbps (%d 系統)'
          % (enc_pkts, h264_mbps, len(node.encoders)))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
