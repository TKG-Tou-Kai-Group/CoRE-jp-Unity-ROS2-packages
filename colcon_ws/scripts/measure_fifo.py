#!/usr/bin/env python3
"""ディスクを FIFO で都度生成・消滅させたときの実時間性を測る。

場に残すのを N 枚に保ち、N 枚を超えたら古いものから消す方式が成立するかを見る。

    python3 scripts/measure_fifo.py --alive 24 --cycles 60

測るのは 4 つ。

  1. スポーンの往復時間
  2. 削除の往復時間。ここが本命。delete_entity は積み荷のディスクに対して
     応答が返らなくなったことがあるので、必ず打ち切り時間を設けて数える。
  3. 回している間のフレーム間隔 (= 操縦映像の引っかかり)
  4. 定常状態で場に N 枚ある状態のフレームレート

積み上げて置いておく方式と違い、FIFO なら場のディスクは接触せず散らばって
静止する。静止した剛体はソルバから外れるので、同じ枚数でも積み荷より
ずっと安い。そこを確かめるのが目的。
"""

import argparse
import collections
import statistics
import threading
import time

import rclpy
from rclpy.node import Node

from rosgraph_msgs.msg import Clock
from simulation_interfaces.msg import Resource, SpawnEntity as SpawnEntityMsg
from simulation_interfaces.srv import SpawnEntities, DeleteEntity

RESULT_OK_VALUES = (0, 1)


class FifoMeasure(Node):

    def __init__(s):
        super().__init__('measure_fifo')
        s.lock = threading.Lock()
        s.frames = []
        s.create_subscription(Clock, '/clock', s.on_clock, 10)
        s.spawner = s.create_client(SpawnEntities, 'spawn_entities')
        s.deleter = s.create_client(DeleteEntity, '/delete_entity')

    def on_clock(s, msg):
        with s.lock:
            s.frames.append(time.time())

    def frame_gaps(s, since):
        with s.lock:
            f = [t for t in s.frames if t >= since]
        return [b - a for a, b in zip(f, f[1:])]

    def _wait(s, future, timeout):
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.002)
        return future.done()

    def spawn(s, name, urdf, pose, timeout=30.0):
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
        fut = s.spawner.call_async(req)
        if not s._wait(fut, timeout):
            return None
        dt = time.time() - t0
        res = fut.result()
        if res is None or res.result.result not in RESULT_OK_VALUES:
            return None
        return dt

    def delete(s, name, timeout=15.0):
        """削除の往復時間。打ち切ったら None を返す (ハングの検出)。"""
        req = DeleteEntity.Request()
        req.entity = name
        t0 = time.time()
        fut = s.deleter.call_async(req)
        if not s._wait(fut, timeout):
            return None
        dt = time.time() - t0
        res = fut.result()
        if res is None or res.result.result not in RESULT_OK_VALUES:
            return None
        return dt


def report(label, values):
    if not values:
        print(f'  {label}  なし')
        return
    v = sorted(x * 1000.0 for x in values)
    print(f'  {label}  中央 {v[len(v) // 2]:.0f} ms / '
          f'最小 {v[0]:.0f} ms / 最大 {v[-1]:.0f} ms')


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', default='/tmp/bench_disc.urdf')
    ap.add_argument('--alive', type=int, default=24,
                    help='場に残す枚数。超えたら古いものから消す')
    ap.add_argument('--cycles', type=int, default=60,
                    help='スポーンする総回数')
    ap.add_argument('--interval', type=float, default=1.0,
                    help='スポーン間隔 [s]')
    ap.add_argument('--prefix', default='fifo')
    ap.add_argument('--baseline', type=float, default=15.0)
    args = ap.parse_args()

    with open(args.urdf) as f:
        urdf = f.read()

    rclpy.init()
    node = FifoMeasure()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    if not node.spawner.wait_for_service(timeout_sec=30.0):
        raise RuntimeError('spawn_entities が出ていない')
    if not node.deleter.wait_for_service(timeout_sec=30.0):
        raise RuntimeError('/delete_entity が出ていない')

    base_from = time.time()
    time.sleep(args.baseline)
    base_gaps = node.frame_gaps(base_from)
    print(f'開始前 ({args.baseline:.0f} s, 場のディスク 0 枚)')
    report('フレーム間隔', base_gaps)

    alive = collections.deque()
    slat, dlat = [], []
    sfail = dfail = dhang = 0

    churn_from = time.time()
    for i in range(args.cycles):
        mark = time.time()
        name = f'{args.prefix}_{i:03d}'
        # 射出方向に散らばる想定で、少しずつ位置をずらして置く
        pose = (-8.0 + 0.35 * (i % 20), -2.0 + 0.4 * ((i // 20) % 5), 0.30)
        dt = node.spawn(name, urdf, pose)
        if dt is None:
            sfail += 1
        else:
            slat.append(dt)
            alive.append(name)

        if len(alive) > args.alive:
            old = alive.popleft()
            dd = node.delete(old)
            if dd is None:
                dhang += 1
            else:
                dlat.append(dd)

        time.sleep(max(0.0, args.interval - (time.time() - mark)))

    time.sleep(1.0)
    churn_gaps = node.frame_gaps(churn_from)

    print(f'\nFIFO {args.alive} 枚を保ちながら {args.cycles} 回 '
          f'(間隔 {args.interval:.1f} s)')
    print(f'  スポーン  成功 {len(slat)}/{args.cycles}'
          + (f' (失敗 {sfail})' if sfail else ''))
    report('  往復    ', slat)
    print(f'  削除      成功 {len(dlat)}/{len(dlat) + dfail + dhang}'
          + (f' / 打ち切り {dhang}' if dhang else ''))
    report('  往復    ', dlat)
    report('フレーム間隔', churn_gaps)

    if base_gaps and churn_gaps:
        b = statistics.median(base_gaps)
        c = statistics.median(churn_gaps)
        print(f'\n  フレーム間隔の中央値 {b * 1000:.0f} ms -> {c * 1000:.0f} ms '
              f'(最悪 {max(churn_gaps) * 1000:.0f} ms)')
    if dhang:
        print(f'  !! 削除が {dhang} 回返ってこなかった。FIFO の実装には'
              f'打ち切りと再試行が要る')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
