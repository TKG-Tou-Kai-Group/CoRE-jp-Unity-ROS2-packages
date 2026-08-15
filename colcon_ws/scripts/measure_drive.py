#!/usr/bin/env python3
"""cmd_vel を出しながら、ground_truth で実際の並進・回転を測る。

走行性能の確認や、物理レート・摩擦設定を変えたときの比較に使う。

    python3 scripts/measure_drive.py --robot sample_robot_1 --vx 0.4 --duration 5
    python3 scripts/measure_drive.py --robot sample_robot_1 --wz 1.0 --duration 5

計測はシミュレーション時間で区切る。物理レートを上げると実時間あたりの進みが
遅くなるので、実時間で区切ると加速区間しか測れずレートを上げるほど遅いという
逆の結果が出る。

yaw は必ずアンラップして積算すること。ground_truth の姿勢から atan2 で出す yaw は
(-pi, pi] に折り返すため、始点と終点の差だけで回転速度を出すと、計測窓の間に
180 度を超えて回った瞬間に符号が反転する。1.0 rad/s を 5 秒なら約 286 度回るので
まず確実に誤る。実際、これに気づかず「旋回の符号が反転する」「3 回に 1 回だけ
逆転する非決定的な不具合」と誤診した経緯がある。指令値を変えても実測の大きさが
変わらないときは、折り返しを疑うこと。
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, Vector3


def wrap(angle):
    """角度を (-pi, pi] へ畳む。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class DriveMeasure(Node):

    def __init__(s, robot, vx, vy, wz):
        super().__init__('measure_drive')
        s.samples = []
        s.create_subscription(PoseStamped, f'/{robot}/ground_truth', s.on_pose, 10)
        s.pub = s.create_publisher(Twist, f'/{robot}/cmd_vel', 10)
        s.cmd = Twist(linear=Vector3(x=vx, y=vy), angular=Vector3(z=wz))
        s.create_timer(0.1, lambda: s.pub.publish(s.cmd))

    def on_pose(s, msg):
        q = msg.pose.orientation
        p = msg.pose.position
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        s.samples.append((stamp, p.x, p.y, p.z, yaw))


def wait_sim_time(node, samples_from, seconds, deadline):
    """シミュレーション時間で seconds ぶん進むまで待つ。"""
    start = node.samples[samples_from][0]
    while node.samples[-1][0] - start < seconds and time.time() < deadline:
        time.sleep(0.2)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot', default='sample_robot_1')
    ap.add_argument('--vx', type=float, default=0.0, help='機体前後 [m/s]')
    ap.add_argument('--vy', type=float, default=0.0, help='機体左右 [m/s]')
    ap.add_argument('--wz', type=float, default=0.0, help='旋回 [rad/s]')
    ap.add_argument('--duration', type=float, default=5.0,
                    help='計測するシミュレーション時間 [s]')
    ap.add_argument('--settle', type=float, default=3.0,
                    help='計測前に助走させるシミュレーション時間 [s]。'
                         'コントローラの加速制限を抜けるため')
    ap.add_argument('--timeout', type=float, default=200.0, help='実時間の打ち切り [s]')
    args = ap.parse_args()

    rclpy.init()
    node = DriveMeasure(args.robot, args.vx, args.vy, args.wz)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    deadline = time.time() + args.timeout
    while not node.samples and time.time() < deadline:
        time.sleep(0.2)
    if not node.samples:
        node.get_logger().error(f'/{args.robot}/ground_truth が来ない')
        rclpy.shutdown()
        sys.exit(1)

    wait_sim_time(node, 0, args.settle, deadline)
    base = len(node.samples) - 1
    wait_sim_time(node, base, args.duration, deadline)

    window = node.samples[base:]
    t0, x0, y0, _, yaw0 = window[0]
    t1, x1, y1, z1, _ = window[-1]
    dt = t1 - t0
    if dt <= 0:
        node.get_logger().error('シミュレーション時間が進まない (停止中か?)')
        rclpy.shutdown()
        sys.exit(1)

    # 回転はサンプル間の差を畳んでから積算する (折り返し対策)
    turned = sum(wrap(window[i][4] - window[i - 1][4]) for i in range(1, len(window)))

    dx, dy = x1 - x0, y1 - y0
    body_x = math.cos(yaw0) * dx + math.sin(yaw0) * dy
    body_y = -math.sin(yaw0) * dx + math.cos(yaw0) * dy

    print(f'指令   vx={args.vx:+.2f} vy={args.vy:+.2f} m/s  '
          f'wz={args.wz:+.2f} rad/s ({math.degrees(args.wz):+.1f} deg/s)')
    print(f'実測   vx={body_x / dt:+.3f} vy={body_y / dt:+.3f} m/s  '
          f'wz={math.degrees(turned) / dt:+.1f} deg/s')
    print(f'       回転量 {math.degrees(turned):+.0f} deg / '
          f'シミュレーション時間 {dt:.1f} s / 車体高さ z={z1:.3f} m')
    for label, cmd, got in (('前後', args.vx, body_x / dt),
                            ('左右', args.vy, body_y / dt),
                            ('旋回', math.degrees(args.wz), math.degrees(turned) / dt)):
        if abs(cmd) > 1e-6:
            print(f'       {label}の達成率 {100.0 * got / cmd:.0f}%')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
