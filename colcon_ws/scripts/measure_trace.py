#!/usr/bin/env python3
"""cmd_vel を出しながら、真値の軌跡を時系列で出す。

達成率だけ見ても「どこで何が起きて遅くなったのか」が分からないときに使う。
車高 z と瞬時速度を並べるので、床の凹凸に乗り上げて減速しているのか、
駆動が出ていないのかを見分けられる。

    python3 scripts/measure_trace.py --robot sample_robot_1 --vy 0.4 --duration 8
"""

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, Vector3


class Trace(Node):

    def __init__(s, robot, vx, vy, wz):
        super().__init__('measure_trace')
        s.lock = threading.Lock()
        s.samples = []
        s.create_subscription(PoseStamped, f'/{robot}/ground_truth', s.on_pose, 10)
        s.pub = s.create_publisher(Twist, f'/{robot}/cmd_vel', 10)
        s.cmd = Twist(linear=Vector3(x=vx, y=vy), angular=Vector3(z=wz))
        s.create_timer(0.05, lambda: s.pub.publish(s.cmd))

    def on_pose(s, msg):
        p = msg.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with s.lock:
            s.samples.append((t, p.x, p.y, p.z))

    def snap(s):
        with s.lock:
            return list(s.samples)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot', default='sample_robot_1')
    ap.add_argument('--vx', type=float, default=0.0)
    ap.add_argument('--vy', type=float, default=0.0)
    ap.add_argument('--wz', type=float, default=0.0)
    ap.add_argument('--duration', type=float, default=8.0,
                    help='計測するシミュレーション時間 [s]')
    ap.add_argument('--timeout', type=float, default=200.0)
    args = ap.parse_args()

    rclpy.init()
    node = Trace(args.robot, args.vx, args.vy, args.wz)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    deadline = time.time() + args.timeout
    while not node.snap() and time.time() < deadline:
        time.sleep(0.2)
    t0 = node.snap()[0][0]
    while node.snap()[-1][0] - t0 < args.duration and time.time() < deadline:
        time.sleep(0.2)

    s = node.snap()
    cmd_speed = math.hypot(args.vx, args.vy)
    print(f'指令 {cmd_speed:.2f} m/s')
    print('   t[s]      x       y       z     速度[m/s]  達成率')
    prev = None
    for cur in s:
        if prev is not None:
            dt = cur[0] - prev[0]
            if dt > 1e-6:
                v = math.hypot(cur[1] - prev[1], cur[2] - prev[2]) / dt
                pct = 100.0 * v / cmd_speed if cmd_speed > 1e-6 else 0.0
                print(f'  {cur[0] - t0:5.2f}  {cur[1]:7.3f} {cur[2]:7.3f} '
                      f'{cur[3]:7.3f}   {v:6.3f}   {pct:5.0f}%')
        prev = cur

    rclpy.shutdown()


if __name__ == '__main__':
    main()
