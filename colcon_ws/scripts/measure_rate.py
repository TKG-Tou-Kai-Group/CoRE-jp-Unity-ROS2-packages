#!/usr/bin/env python3
"""RTF (シミュレーション時間の進み) と、Unity のフレームレートを測る。

physics_hz を変えたときの比較に使う。

    python3 scripts/measure_rate.py --duration 20

Unity のセンサは Update() すなわち描画フレームごとに publish されるので、
センサトピックのレートがそのまま Unity のフレームレートになる。画像そのものを
購読すると 2.76MB/枚 の展開コストが測定側に乗るため、同じセンサから同じレートで
出る camera_info を数える。/clock も 1 フレームに 1 回なので、レート測定の
対象にもなるし、値の差分から RTF も出せる。

RTF はシミュレーション時間の増分 / 実時間の増分。GUI の表示と同じ量を
外から数値で取るためのもの。
"""

import argparse
import threading
import time

import rclpy
from rclpy.node import Node

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo


class RateMeasure(Node):

    def __init__(s, topics):
        super().__init__('measure_rate')
        s.lock = threading.Lock()
        s.clock = []          # (wall, sim)
        s.hits = {t: [] for t in topics}
        s.create_subscription(Clock, '/clock', s.on_clock, 10)
        for t in topics:
            s.create_subscription(
                CameraInfo, t, lambda m, t=t: s.on_info(t), 10)

    def on_clock(s, msg):
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        with s.lock:
            s.clock.append((time.time(), sim))

    def on_info(s, topic):
        with s.lock:
            s.hits[topic].append(time.time())


def span(samples):
    """計測窓の中に入っているぶんだけを返す。"""
    return samples[0], samples[-1]


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robots', nargs='*', default=['sample_robot_1'],
                    help='フレームレートを見るロボット名')
    ap.add_argument('--duration', type=float, default=20.0,
                    help='計測する実時間 [s]')
    ap.add_argument('--settle', type=float, default=5.0,
                    help='計測前に捨てる実時間 [s]')
    args = ap.parse_args()

    topics = [f'/{r}/camera_link/camera_info' for r in args.robots]

    rclpy.init()
    node = RateMeasure(topics)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    time.sleep(args.settle)
    with node.lock:
        clock_base = len(node.clock)
        hit_base = {t: len(v) for t, v in node.hits.items()}

    time.sleep(args.duration)

    with node.lock:
        clock = node.clock[clock_base:]
        hits = {t: v[hit_base[t]:] for t, v in node.hits.items()}

    print(f'計測 {args.duration:.0f} s (実時間)')

    if len(clock) >= 2:
        (w0, s0), (w1, s1) = span(clock)
        wall = w1 - w0
        sim = s1 - s0
        print(f'  RTF           {sim / wall:.3f}   '
              f'(シミュレーション時間 {sim:.2f} s / 実時間 {wall:.2f} s)')
        print(f'  /clock        {(len(clock) - 1) / wall:.2f} Hz')
    else:
        print('  RTF           測定不能 (/clock が来ない。停止中か?)')

    for t in topics:
        v = hits[t]
        if len(v) >= 2:
            w0, w1 = span(v)
            print(f'  {t}  {(len(v) - 1) / (w1 - w0):.2f} FPS')
        else:
            print(f'  {t}  測定不能')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
