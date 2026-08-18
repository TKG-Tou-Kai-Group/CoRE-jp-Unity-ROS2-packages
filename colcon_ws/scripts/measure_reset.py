#!/usr/bin/env python3
"""試合のリセットが期待どおり効いているかを測る。

    python3 scripts/measure_reset.py --robots sample_robot_1 sample_robot_2

リセットは 2 系統ある。

  reset_simulation (SCOPE_STATE=2)  シミュレータ側。エンティティを初期姿勢へ戻し、
                                    接触記録も消す。scope はビット和で、
                                    1 は SCOPE_TIME (時刻のみ) なので注意
  /reset_hp (std_msgs/Bool)       hp_manager 側。HP を初期値へ戻す

競技中に効いてほしいのは「機体が開始位置へ戻る」「HP が満タンに戻る」
「ディスクが装填し直される」「試合の進行状態が壊れない」の 4 点なので、
リセット前後で次を突き合わせる。

  - 各機体の位置と姿勢
  - 各機体の HP
  - 場に残っているディスクの数と位置
  - シミュレーションの実行状態 (リセットで停止してしまわないか)

reset_simulation がディスクを装填し直すかどうかは仕様として決まっていないので、
「どうなるか」を観測して報告する。
"""

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32
from simulation_interfaces.srv import (
    GetEntitiesStates, GetSimulationState, ResetSimulation)

RESULT_OK_VALUES = (0, 1)
STATE_NAMES = {0: 'STOPPED', 1: 'PLAYING', 2: 'PAUSED', 3: 'QUITTING'}


class ResetMeasure(Node):

    def __init__(s, robots):
        super().__init__('measure_reset')
        s.lock = threading.Lock()
        s.pose = {}
        s.hp = {}
        for name in robots:
            s.create_subscription(
                PoseStamped, f'/{name}/ground_truth',
                lambda m, n=name: s.on_pose(n, m), 10)
            s.create_subscription(
                Int32, f'/{name}/robot_hp',
                lambda m, n=name: s.on_hp(n, m), 10)
        s.reset_hp_pub = s.create_publisher(Bool, '/reset_hp', 10)
        s.reset = s.create_client(ResetSimulation, '/reset_simulation')
        s.states = s.create_client(GetEntitiesStates, '/get_entities_states')
        s.sim_state = s.create_client(GetSimulationState, '/get_simulation_state')

    def on_pose(s, name, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with s.lock:
            s.pose[name] = (p.x, p.y, p.z, yaw)

    def on_hp(s, name, msg):
        with s.lock:
            s.hp[name] = msg.data

    def snap(s):
        with s.lock:
            return dict(s.pose), dict(s.hp)

    def _wait(s, fut, timeout):
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            time.sleep(0.02)
        return fut.done()

    def discs(s, timeout=25.0):
        req = GetEntitiesStates.Request()
        req.filters.filter = '.*flying_disc.*'
        fut = s.states.call_async(req)
        if not s._wait(fut, timeout):
            return None
        res = fut.result()
        if res is None:
            return None
        return {n: (st.pose.position.x, st.pose.position.y, st.pose.position.z)
                for n, st in zip(res.entities, res.states)}

    def state(s, timeout=20.0):
        fut = s.sim_state.call_async(GetSimulationState.Request())
        if not s._wait(fut, timeout):
            return None
        res = fut.result()
        return res.state.state if res is not None else None

    def do_reset(s, scope, timeout=60.0):
        req = ResetSimulation.Request()
        req.scope = scope
        fut = s.reset.call_async(req)
        if not s._wait(fut, timeout):
            return '応答なし (打ち切り)'
        res = fut.result()
        if res is None:
            return '応答なし'
        if res.result.result not in RESULT_OK_VALUES:
            return f'result={res.result.result} {res.result.error_message}'
        return None


def show(label, pose, hp, discs, state):
    print(f'  [{label}]')
    for name in sorted(pose):
        x, y, z, yaw = pose[name]
        print(f'    {name:16s} 位置 ({x:+7.2f},{y:+7.2f},{z:+6.3f}) '
              f'yaw {math.degrees(yaw):+7.1f} deg   HP {hp.get(name)}')
    n = '取得失敗' if discs is None else len(discs)
    print(f'    ディスク数 {n}   実行状態 {STATE_NAMES.get(state, state)}')


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robots', nargs='+', default=['sample_robot_1'])
    # SCOPE_TIME=1 / SCOPE_STATE=2 / SCOPE_SPAWNED=4 / SCOPE_ALL=255 のビット和。
    # 姿勢を戻したいので既定は 2。1 を渡すと時刻だけ戻り、姿勢は動かない。
    ap.add_argument('--scope', type=int, default=2,
                    help='ResetSimulation の scope (ビット和)。'
                         '1=TIME 2=STATE 4=SPAWNED 255=ALL。既定 2 = STATE')
    ap.add_argument('--settle', type=float, default=8.0,
                    help='リセット後に落ち着くまで待つ実時間 [s]')
    args = ap.parse_args()

    rclpy.init()
    node = ResetMeasure(args.robots)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    for cli, name in ((node.reset, '/reset_simulation'),
                      (node.states, '/get_entities_states'),
                      (node.sim_state, '/get_simulation_state')):
        if not cli.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f'{name} が出ていない')

    deadline = time.time() + 60
    while time.time() < deadline:
        pose, hp = node.snap()
        if all(r in pose for r in args.robots):
            break
        time.sleep(0.5)

    pose, hp = node.snap()
    print('リセットの確認')
    show('リセット前', pose, hp, node.discs(), node.state())

    err = node.do_reset(args.scope)
    if err:
        print(f'\n  !! reset_simulation が失敗: {err}')
    else:
        print(f'\n  reset_simulation(scope={args.scope}) 成功')
    time.sleep(args.settle)

    pose2, hp2 = node.snap()
    show('reset_simulation の後', pose2, hp2, node.discs(), node.state())

    # HP はシミュレータ側では戻らない。hp_manager へ別途投げる必要がある。
    node.reset_hp_pub.publish(Bool(data=True))
    time.sleep(3.0)
    pose3, hp3 = node.snap()
    show('/reset_hp の後', pose3, hp3, node.discs(), node.state())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
