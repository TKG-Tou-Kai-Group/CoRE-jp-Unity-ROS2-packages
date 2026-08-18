#!/usr/bin/env python3
"""ディスクを 1 枚ずつスポーンして、遅延とフレームの引っかかりを測る。

射出のたびにディスクを生成する方式が成立するかの判断材料にする。

    python3 scripts/measure_spawn.py --count 20

見るのは 2 つ。

  1. スポーン 1 回あたりの往復時間。射出の間隔より十分短いか。
  2. スポーンした瞬間にフレームが飛ばないか。遅延が短くても、その 1 フレーム
     だけ数百 ms 止まるなら操縦映像はカクつく。/clock はシミュレータの
     フレームごとに出るので、その間隔の最大値を平常時と比べれば分かる。

あらかじめ 20 枚積んでおく方式と違い、都度生成なら積み荷ぶんの剛体が
常時 0 枚になる。積み荷は接触したまま積み重なっているのでソルバのコストが
高く、これがフレームレートを律速していた (bench_disc_count.sh を参照)。
"""

import argparse
import statistics
import threading
import time

import rclpy
from rclpy.node import Node

from rosgraph_msgs.msg import Clock
from simulation_interfaces.msg import Resource, SpawnEntity as SpawnEntityMsg
from simulation_interfaces.srv import SpawnEntities

RESULT_OK_VALUES = (0, 1)


class SpawnMeasure(Node):

    def __init__(s):
        super().__init__('measure_spawn')
        s.lock = threading.Lock()
        s.frames = []          # /clock を受けた実時刻
        s.create_subscription(Clock, '/clock', s.on_clock, 10)
        s.client = s.create_client(SpawnEntities, 'spawn_entities')

    def on_clock(s, msg):
        with s.lock:
            s.frames.append(time.time())

    def frame_gaps(s, since):
        with s.lock:
            f = [t for t in s.frames if t >= since]
        return [b - a for a, b in zip(f, f[1:])]

    def spawn_one(s, name, urdf, pose, timeout=30.0):
        """1 枚だけスポーンして、往復時間を返す。失敗なら None。"""
        req = SpawnEntities.Request()
        e = SpawnEntityMsg()
        e.name = name
        e.allow_renaming = False
        e.entity_resource = Resource()
        e.entity_resource.resource_string = urdf
        e.entity_namespace = ''
        e.initial_pose.header.frame_id = ''
        e.initial_pose.pose.position.x = pose[0]
        e.initial_pose.pose.position.y = pose[1]
        e.initial_pose.pose.position.z = pose[2]
        e.initial_pose.pose.orientation.w = 1.0
        req.spawn_requests.append(e)

        t0 = time.time()
        future = s.client.call_async(req)
        deadline = t0 + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.002)
        if not future.done():
            return None
        dt = time.time() - t0
        res = future.result()
        if res is None or res.result.result not in RESULT_OK_VALUES:
            msg = res.result.error_message if res else 'no response'
            s.get_logger().error(f'{name}: {msg}')
            return None
        return dt


def report(label, values, unit='ms', scale=1000.0):
    if not values:
        print(f'  {label}  測定不能')
        return
    v = sorted(x * scale for x in values)
    mid = len(v) // 2
    print(f'  {label}  中央 {v[mid]:.0f} {unit} / '
          f'最小 {v[0]:.0f} {unit} / 最大 {v[-1]:.0f} {unit}')


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', default='/tmp/bench_disc.urdf',
                    help='スポーンするディスクの URDF')
    ap.add_argument('--count', type=int, default=20, help='スポーンする枚数')
    ap.add_argument('--prefix', default='bench_spawn',
                    help='エンティティ名の接頭辞。allow_renaming を立てていないので、'
                         '同じシミュレータへ2回流すときは必ず変えること '
                         '(変えないと NAME_NOT_UNIQUE で全数失敗する)')
    ap.add_argument('--interval', type=float, default=1.0,
                    help='スポーン間隔 [s]。射出の間隔を想定')
    ap.add_argument('--baseline', type=float, default=15.0,
                    help='平常時のフレーム間隔を測る時間 [s]')
    args = ap.parse_args()

    with open(args.urdf) as f:
        urdf = f.read()

    rclpy.init()
    node = SpawnMeasure()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    if not node.client.wait_for_service(timeout_sec=30.0):
        raise RuntimeError('spawn_entities が出ていない')

    # --- 平常時のフレーム間隔 -------------------------------------------
    base_from = time.time()
    time.sleep(args.baseline)
    base_gaps = node.frame_gaps(base_from)
    print(f'スポーン前 ({args.baseline:.0f} s)')
    report('フレーム間隔  ', base_gaps)

    # --- 1 枚ずつスポーン -----------------------------------------------
    lat = []
    failed = []
    spawn_from = time.time()
    for i in range(args.count):
        mark = time.time()
        dt = node.spawn_one(f'{args.prefix}_{i:03d}', urdf,
                            (-8.0 + 0.4 * i, 0.0, 0.30))
        if dt is None:
            failed.append(i)
        else:
            lat.append(dt)
        time.sleep(max(0.0, args.interval - (time.time() - mark)))
    # 最後のスポーンぶんもフレーム間隔に入るよう、少しだけ余分に見る
    time.sleep(1.0)
    spawn_gaps = node.frame_gaps(spawn_from)

    print(f'\n1 枚ずつ {args.count} 回スポーン '
          f'(間隔 {args.interval:.1f} s), 成功 {len(lat)}/{args.count}')
    report('スポーン往復  ', lat)
    report('フレーム間隔  ', spawn_gaps)
    if failed:
        print(f'  失敗 {len(failed)} 件 (名前の重複なら --prefix を変えること)')

    if base_gaps and spawn_gaps:
        b = statistics.median(base_gaps)
        h = max(spawn_gaps)
        print(f'\n  平常時の中央値 {b * 1000:.0f} ms に対し、'
              f'スポーン中の最悪フレーム間隔は {h * 1000:.0f} ms '
              f'({h / b:.1f} 倍)')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
