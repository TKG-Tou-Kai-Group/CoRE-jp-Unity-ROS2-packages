#!/usr/bin/env python3
"""1 台を前進させて他機にぶつけ、押された側が動くかを測る。

BodyTwistDrive を「速度代入」ではなく「力で追従」にした理由が満たされている
かの確認。速度代入の実装だと、押された側は指令速度 (0) を維持し続けるので
まったく動かない。力で追従させていれば、押されている間は指令から外れて動く。

    python3 scripts/measure_push.py --pusher sample_robot_1 --target sample_robot_2

判定は「押された側の移動量」。押し出せていれば数十 cm 動く。
押している側が停止する (指令速度を維持できない) ことも併せて見る。
"""

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, Vector3


class PushMeasure(Node):

    def __init__(s, pusher, target, vx):
        super().__init__('measure_push')
        s.lock = threading.Lock()
        s.pose = {pusher: [], target: []}
        for name in (pusher, target):
            s.create_subscription(
                PoseStamped, f'/{name}/ground_truth',
                lambda m, n=name: s.on_pose(n, m), 10)
        s.pub = s.create_publisher(Twist, f'/{pusher}/cmd_vel', 10)
        s.cmd = Twist(linear=Vector3(x=vx))
        s.driving = False
        s.create_timer(0.05, s.tick)

    def tick(s):
        if s.driving:
            s.pub.publish(s.cmd)

    def on_pose(s, name, msg):
        p = msg.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with s.lock:
            s.pose[name].append((t, p.x, p.y))

    def snap(s, name):
        with s.lock:
            return list(s.pose[name])


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pusher', default='sample_robot_1')
    ap.add_argument('--target', default='sample_robot_2')
    ap.add_argument('--vx', type=float, default=1.0)
    ap.add_argument('--duration', type=float, default=8.0,
                    help='押し続けるシミュレーション時間 [s]')
    args = ap.parse_args()

    rclpy.init()
    node = PushMeasure(args.pusher, args.target, args.vx)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    deadline = time.time() + 60
    while (not node.snap(args.pusher) or not node.snap(args.target)) \
            and time.time() < deadline:
        time.sleep(0.2)
    if not node.snap(args.target):
        print('  ground_truth が来ない')
        rclpy.shutdown()
        return

    start = {n: node.snap(n)[-1] for n in (args.pusher, args.target)}
    t0 = start[args.pusher][0]

    node.driving = True
    while node.snap(args.pusher)[-1][0] - t0 < args.duration:
        time.sleep(0.1)
    node.driving = False

    end = {n: node.snap(n)[-1] for n in (args.pusher, args.target)}

    def moved(n):
        return math.hypot(end[n][1] - start[n][1], end[n][2] - start[n][2])

    gap0 = math.hypot(start[args.pusher][1] - start[args.target][1],
                      start[args.pusher][2] - start[args.target][2])
    gap1 = math.hypot(end[args.pusher][1] - end[args.target][1],
                      end[args.pusher][2] - end[args.target][2])

    print(f'{args.pusher} に vx={args.vx} m/s を {args.duration:.0f} s 指令')
    print(f'  押した側の移動   {moved(args.pusher):.2f} m')
    print(f'  押された側の移動 {moved(args.target):.2f} m')
    print(f'  機体間の距離     {gap0:.2f} m -> {gap1:.2f} m')
    if moved(args.target) > 0.1:
        print('  => 押し合いが成立している (力で追従できている)')
    else:
        print('  => 押された側が動かない。速度代入になっていないか要確認')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
