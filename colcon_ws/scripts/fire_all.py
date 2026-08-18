#!/usr/bin/env python3
"""指定した機体すべてから同時に撃ち続ける。8 台構成の負荷試験用。

teleop と同じ velocity_controller/commands を出す。装填は押し出しと引き戻しを
交互に繰り返す (prismatic なので 1 往復 1 枚)。
"""

import argparse
import threading
import time

import rclpy
from std_msgs.msg import Float64MultiArray

WHEEL = 500.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robots', nargs='+', required=True)
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--push', type=float, default=0.6)
    ap.add_argument('--retract', type=float, default=1.6)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('fire_all')
    pubs = [node.create_publisher(
        Float64MultiArray, f'/{r}/velocity_controller/commands', 10)
        for r in args.robots]
    state = {'loader': -1.0}

    def tick():
        msg = Float64MultiArray()
        msg.data = [WHEEL, WHEEL, state['loader']]
        for p in pubs:
            p.publish(msg)

    node.create_timer(0.05, tick)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    end = time.time() + args.duration
    strokes = 0
    while time.time() < end:
        state['loader'] = 1.0
        time.sleep(args.push)
        state['loader'] = -1.0
        time.sleep(args.retract)
        strokes += 1
    state['loader'] = -1.0
    time.sleep(0.5)
    print(f'{len(args.robots)} 台から {strokes} ストロークずつ射撃した')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
