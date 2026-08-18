#!/usr/bin/env python3
"""射出機構の性能を、ディスクの真値状態で測る。

物理レートや接触設定を変えたときに、ちゃんと装填・射出できているかの確認に使う。

    python3 scripts/measure_shoot.py --robot sample_robot_1 --duration 20

velocity_controller/commands は teleop と同じ [射出輪, 射出輪, 装填] の 3 要素で、
射出中は [800, 800, 1]、待機は [800, 800, -1]。まず射出輪だけ回して定常まで
待ってから装填を始める (実機同様、輪が上がりきる前に送ると初速が出ない)。

装填は回転ではなく prismatic (直動、ストローク 0.2 m) で、1 往復につき 1 枚しか
送れない。teleop がボタンの押下で +1、解放で -1 を出しているのと同じで、
+1 を出しっぱなしにすると押し切ったところで止まり、何秒待っても 2 枚目は出ない。
そのため押し出しと引き戻しを交互に繰り返す。

計測はシミュレーション時間で区切る。実時間で区切ると、物理レートを変えたときに
「射出できた枚数」がフレームレートの差で変わってしまい比較にならない。

ディスクは試験中に現れたものを全部追跡する。開始時のスナップショットだけを見る
作りにしていると、都度生成 (flying_disc_feeder) では名前が毎回変わるため、
最初の 1 枚しか数えられない (実測では 28 ストロークで「0/1」と出た。実際には
初速 30.8 m/s で正常に射出されていた)。

初速は GetEntitiesStates の twist (真値) をポーリングした最大値。ポーリングは
Unity のフレームレートが上限なので、フレームレートが低い条件ほど飛翔中の
サンプルが減り、ピークを取り逃して低めに出る。初速どうしを直接比べるときは
併記されるサンプル数を必ず見ること。装填の成否 (射出枚数) と到達距離は
サンプリングに依存しないので、条件間の比較にはそちらを使う。
"""

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from simulation_interfaces.srv import GetEntitiesStates

# 射出輪の指令速度 [rad/s]。teleop の shoot_wheel_velocity と揃えること。
# 初速 = 射出輪の半径 0.04 x この値。500 で約 20 m/s。
WHEEL_VELOCITY = 500.0
LOADER_FEED = 1.0
LOADER_IDLE = -1.0

LAUNCHED_DISTANCE = 1.0   # これだけ動いたら「射出された」とみなす [m]


class ShootMeasure(Node):

    def __init__(s, robot):
        super().__init__('measure_shoot')
        s.robot = robot
        s.lock = threading.Lock()
        s.sim_time = None
        s.wheel_vel = []

        s.create_subscription(Clock, '/clock', s.on_clock, 10)
        s.create_subscription(
            JointState, f'/{robot}/joint_states', s.on_joints, 10)
        s.cmd = s.create_publisher(
            Float64MultiArray, f'/{robot}/velocity_controller/commands', 10)
        # 走行しながら撃つとき用。指令が途切れると body_twist_drive の
        # command_timeout で止まるので、試験中は出し続ける。
        s.vel = s.create_publisher(Twist, f'/{robot}/cmd_vel', 10)
        s.drive_cmd = None
        s.states = s.create_client(GetEntitiesStates, '/get_entities_states')

        s.loader = LOADER_IDLE
        s.create_timer(0.1, s.publish_cmd)

    def publish_cmd(s):
        msg = Float64MultiArray()
        msg.data = [WHEEL_VELOCITY, WHEEL_VELOCITY, s.loader]
        s.cmd.publish(msg)
        if s.drive_cmd is not None:
            s.vel.publish(s.drive_cmd)

    def on_clock(s, msg):
        with s.lock:
            s.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_joints(s, msg):
        """射出輪が指令値まで上がっているかを見る。"""
        for name, vel in zip(msg.name, msg.velocity):
            if name == 'shooter_wheel1_link_joint':
                with s.lock:
                    s.wheel_vel.append(vel)

    def now(s):
        with s.lock:
            return s.sim_time

    def wait_sim(s, seconds, timeout=300.0):
        """シミュレーション時間で seconds ぶん進むまで待つ。"""
        deadline = time.time() + timeout
        while s.now() is None and time.time() < deadline:
            time.sleep(0.1)
        start = s.now()
        if start is None:
            raise RuntimeError('/clock が来ない (シミュレータが停止中か?)')
        while time.time() < deadline:
            if s.now() - start >= seconds:
                return
            time.sleep(0.05)
        raise RuntimeError('シミュレーション時間が進まない (停止中か?)')

    def disc_states(s, timeout=10.0):
        """ロボットのディスク全枚ぶんの真値状態を 1 回の呼び出しで取る。"""
        req = GetEntitiesStates.Request()
        req.filters.filter = f'^{s.robot}_flying_disc_[0-9]+$'
        future = s.states.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            return {}
        res = future.result()
        out = {}
        for name, st in zip(res.entities, res.states):
            p = st.pose.position
            v = st.twist.linear
            out[name] = ((p.x, p.y, p.z),
                         math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z))
        return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot', default='sample_robot_1')
    ap.add_argument('--duration', type=float, default=20.0,
                    help='装填を往復させるシミュレーション時間 [s]')
    ap.add_argument('--push', type=float, default=1.5,
                    help='1 回の押し出しにかけるシミュレーション時間 [s]')
    ap.add_argument('--retract', type=float, default=1.5,
                    help='1 回の引き戻しにかけるシミュレーション時間 [s]')
    ap.add_argument('--spinup', type=float, default=3.0,
                    help='射出輪だけ回して待つシミュレーション時間 [s]')
    ap.add_argument('--settle', type=float, default=5.0,
                    help='射出後、ディスクが止まるまで待つシミュレーション時間 [s]')
    ap.add_argument('--vx', type=float, default=0.0, help='走行しながら撃つ: 前後 [m/s]')
    ap.add_argument('--vy', type=float, default=0.0, help='走行しながら撃つ: 左右 [m/s]')
    ap.add_argument('--wz', type=float, default=0.0, help='走行しながら撃つ: 旋回 [rad/s]')
    args = ap.parse_args()

    rclpy.init()
    node = ShootMeasure(args.robot)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    if args.vx or args.vy or args.wz:
        node.drive_cmd = Twist(linear=Vector3(x=args.vx, y=args.vy),
                               angular=Vector3(z=args.wz))

    if not node.states.wait_for_service(timeout_sec=30.0):
        raise RuntimeError('/get_entities_states が出ていない')

    node.wait_sim(0.5)
    start = node.disc_states()
    if start is None:
        raise RuntimeError('ディスクの状態を取得できない')

    # 射出輪を定常まで上げてから装填を始める
    node.wait_sim(args.spinup)
    with node.lock:
        node.wheel_vel.clear()
    spinup_samples = None

    # name -> [初めて見た位置, 最後に見た位置, 最大速度]
    # 途中で生成されたものも、FIFO で消えたものも取りこぼさないようにする。
    seen = {}

    def observe(states):
        for name, (pos, speed) in states.items():
            rec = seen.get(name)
            if rec is None:
                seen[name] = [pos, pos, speed]
            else:
                rec[1] = pos
                rec[2] = max(rec[2], speed)

    observe(start)
    samples = 0
    strokes = 0
    t0 = node.now()
    phase_start = t0
    node.loader = LOADER_FEED

    while True:
        now = node.now()
        if now - t0 >= args.duration:
            break
        # 押し出しと引き戻しを交互に。境目はシミュレーション時間で決めるので、
        # フレームレートが低い条件でもストローク数は揃う。
        limit = args.push if node.loader == LOADER_FEED else args.retract
        if now - phase_start >= limit:
            if node.loader == LOADER_FEED:
                node.loader = LOADER_IDLE
            else:
                node.loader = LOADER_FEED
                strokes += 1
            phase_start = now
        cur = node.disc_states()
        if cur:
            samples += 1
            observe(cur)

    node.loader = LOADER_IDLE
    with node.lock:
        wheel = list(node.wheel_vel)
    spinup_samples = len(wheel)

    node.wait_sim(args.settle)
    end = node.disc_states()
    if end:
        observe(end)

    launched = []
    for name in sorted(seen):
        first, last, peak_speed = seen[name]
        moved = math.dist(first, last)
        if moved >= LAUNCHED_DISTANCE:
            launched.append((name, moved, peak_speed))

    motion = ('静止' if node.drive_cmd is None else
              f'走行中 vx={args.vx} vy={args.vy} wz={args.wz}')
    print(f'ロボット {args.robot} / 装填 {args.duration:.0f} s '
          f'(シミュレーション時間) / {strokes} ストローク / {motion}')
    print(f'  射出できた枚数   {len(launched)} / 観測 {len(seen)} 枚 '
          f'({100.0 * len(launched) / max(strokes, 1):.0f}% / ストローク)')
    if wheel:
        print(f'  射出輪の実測速度 {sum(wheel) / len(wheel):+.1f} rad/s '
              f'(指令 {WHEEL_VELOCITY:.0f}, '
              f'達成率 {100.0 * abs(sum(wheel) / len(wheel)) / WHEEL_VELOCITY:.0f}%, '
              f'{spinup_samples} サンプル)')
    if launched:
        dists = sorted(d for _, d, _ in launched)
        speeds = sorted(v for _, _, v in launched)
        mid = len(dists) // 2
        # 射出輪の周速 (roller_radius 0.04 x 800 rad/s = 32 m/s) を大きく超える
        # 弾は、貫入の解消で過大な力積が出た破綻。正常な射出とは分けて数える。
        blown = [v for v in speeds if v > 50.0]
        print(f'  到達距離         中央 {dists[mid]:.2f} m / '
              f'最小 {dists[0]:.2f} m / 最大 {dists[-1]:.2f} m')
        print(f'  観測最大速度     中央 {speeds[mid]:.2f} m/s / '
              f'最小 {speeds[0]:.2f} m/s / 最大 {speeds[-1]:.2f} m/s '
              f'(理論 32 m/s, {samples} 回ポーリング)')
        if blown:
            print(f'  !! 物理破綻      {len(blown)} 発が 50 m/s 超 '
                  f'(最大 {max(blown):.0f} m/s)')
    else:
        print('  1 枚も射出されていない')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
