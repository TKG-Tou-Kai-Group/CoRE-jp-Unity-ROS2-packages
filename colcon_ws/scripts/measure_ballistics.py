#!/usr/bin/env python3
"""射出されたディスクの弾道を測り、装甲板に当たる距離帯を出す。

    python3 scripts/measure_ballistics.py --robot sample_robot_1 --shots 12

装甲板は base_link の +0.45 中心、0.15 角なので帯は +0.375〜+0.525。
射出口 (射出輪) は +0.505 にある。

測り方で踏んだ落とし穴が 3 つあるので、同じ間違いをしないこと。

  飛翔中だけを使う
      着地した弾も同じ距離のビンに入る。速度で絞らないと、撃った弾が最終的に
      全部地面へ落ちるぶんで中央値が埋まり「2 m 以降は高さ 0.01」という
      でたらめな表になる。

  距離は機体正面への符号付き射影で取る
      符号なしの距離だと、後ろや横へ飛んだ弾も前方の弾と同じビンに入って
      弾道が混ざる。横ずれも併せて出し、狙いのばらつきが見えるようにする。

  1 回の結果で判断しない
      同じ条件でも飛翔数が 0〜8 発と大きく振れる。--repeat で複数回まわし、
      回ごとの値を並べて見ること。

  機体が接地しているか確かめる
      スポーン後にフィールド構造物へ乗り上げることがある。接地時の base_link は
      z = 0.005 前後だが、乗り上げると 0.14 まで上がり、その状態では装填口の
      位置がずれて 1 発も射出できない。測る前に高さを見ること。
"""

import argparse
import math
import statistics
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from simulation_interfaces.srv import GetEntitiesStates

# 射出輪の半径 [m]。初速 = この値 x 射出輪の指令速度 [rad/s]。
ROLLER_RADIUS = 0.04
# 射出輪の指令速度 [rad/s] の既定。teleop の shoot_wheel_velocity と揃えること。
# 250 で約 10 m/s、500 で約 20 m/s、800 で約 32 m/s。--wheel で変えられる。
WHEEL_VELOCITY = 500.0
ARMOR_LOW, ARMOR_HIGH = 0.375, 0.525   # base_link 基準の装甲板の帯
MUZZLE_Z = 0.505
# 接地しているとみなす base_link の高さ [m]。これより高いと構造物の上。
GROUNDED_Z = 0.05
# 飛翔中とみなす速さ [m/s]。これ未満は着地済み・待機中として弾道から除く。
FLYING_SPEED = 3.0
# 射出輪の周速。これを大きく超える弾は貫入の解消で出た物理破綻。
# main() で --wheel に合わせて入れ直す。
THEORY_SPEED = ROLLER_RADIUS * WHEEL_VELOCITY
BLOWUP_SPEED = THEORY_SPEED * 1.6
# 初速とみなす範囲。サンプリングはフレームレート律速なので、遠くのサンプルは
# 壁や地面で跳ね返った後になる。射出直後に限らないと方位が滅茶苦茶になる
# (跳ね返りを初速として数えて、方位の幅が ±180 度に見えていた)。
LAUNCH_MAX_DIST = 1.5
# 射出口の高さからこれ以上外れたサンプルは、跳ねた後とみなして初速に使わない。
LAUNCH_MAX_DROP = 0.15


class Ballistics(Node):

    def __init__(s, robot):
        super().__init__('measure_ballistics')
        s.lock = threading.Lock()
        s.pose = None
        s.robot = robot
        s.create_subscription(PoseStamped, f'/{robot}/ground_truth', s.on_pose, 10)
        s.cmd = s.create_publisher(
            Float64MultiArray, f'/{robot}/velocity_controller/commands', 10)
        s.states = s.create_client(GetEntitiesStates, '/get_entities_states')
        s.loader = -1.0
        s.create_timer(0.05, lambda: s.cmd.publish(
            Float64MultiArray(data=[WHEEL_VELOCITY, WHEEL_VELOCITY, s.loader])))
        # WHEEL_VELOCITY は main() で --wheel に差し替わる。lambda は呼び出し時に
        # 読むので、差し替え後の値が使われる。

    def on_pose(s, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with s.lock:
            s.pose = (p.x, p.y, p.z, yaw)

    def discs(s, timeout=10.0):
        req = GetEntitiesStates.Request()
        req.filters.filter = f'^{s.robot}_flying_disc_.*$'
        fut = s.states.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            time.sleep(0.005)
        if not fut.done():
            return {}
        res = fut.result()
        return {n: ((st.pose.position.x, st.pose.position.y, st.pose.position.z),
                    (st.twist.linear.x, st.twist.linear.y, st.twist.linear.z))
                for n, st in zip(res.entities, res.states)}


def one_run(node, shots, push, retract):
    """1 回ぶん撃って、弾ごとの軌跡と射出直後の速度を返す。"""
    while node.pose is None:
        time.sleep(0.2)
    rx, ry, rz, yaw = node.pose
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    tracks = {}   # name -> [(前方, 横, 高さ)] 飛翔中のみ
    launch = {}   # name -> (前方速度, 横速度, 鉛直速度)

    def sample():
        for n, (p, v) in node.discs().items():
            speed = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
            if speed < FLYING_SPEED:
                continue
            dx, dy = p[0] - rx, p[1] - ry
            fwd = dx * cos_y + dy * sin_y     # 機体正面が正
            lat = -dx * sin_y + dy * cos_y    # 機体左が正
            tracks.setdefault(n, []).append((fwd, lat, p[2] - rz))
            vf = v[0] * cos_y + v[1] * sin_y
            vl = -v[0] * sin_y + v[1] * cos_y
            # 射出直後だけを初速とする。遠い・高さがずれたサンプルは
            # 跳ね返った後なので混ぜない。
            height = p[2] - rz
            if (n not in launch
                    and 0.2 < fwd < LAUNCH_MAX_DIST
                    and abs(height - MUZZLE_Z) < LAUNCH_MAX_DROP
                    and math.hypot(vf, vl) > THEORY_SPEED * 0.3):
                launch[n] = (vf, vl, v[2])

    time.sleep(2.0)
    for _ in range(shots):
        node.loader = 1.0
        end = time.time() + push
        while time.time() < end:
            sample()
            time.sleep(0.02)
        node.loader = -1.0
        end = time.time() + retract
        while time.time() < end:
            sample()
            time.sleep(0.02)
    node.loader = -1.0
    return tracks, launch


def summarise(tracks, launch, shots, label):
    flown = {n: t for n, t in tracks.items()
             if t and max(abs(f) for f, _, _ in t) > 1.0}
    backward = [n for n, t in flown.items() if min(f for f, _, _ in t) < -1.0]

    print(f'--- {label} ---')
    print(f'  飛翔した弾 {len(flown)} 発 / {shots} ストローク  '
          f'(後ろへ飛んだ弾 {len(backward)} 発)')
    if not launch:
        print('  射出直後を捕まえられなかった')
        return None

    vals = list(launch.values())
    elev = sorted(math.degrees(math.atan2(vz, math.hypot(vf, vl)))
                  for vf, vl, vz in vals)
    azim = sorted(math.degrees(math.atan2(vl, vf)) for vf, vl, vz in vals)
    speeds = sorted(math.sqrt(vf ** 2 + vl ** 2 + vz ** 2) for vf, vl, vz in vals)
    fwd_ok = sum(1 for vf, _, _ in vals if vf > 0)
    blown = [v for v in speeds if v > BLOWUP_SPEED]
    mid = len(elev) // 2
    print(f'  前向きに射出 {fwd_ok}/{len(vals)} 発')
    print(f'  仰角  中央 {elev[mid]:+6.1f} deg  範囲 {elev[0]:+.1f}〜{elev[-1]:+.1f}')
    print(f'  方位  中央 {azim[mid]:+6.1f} deg  範囲 {azim[0]:+.1f}〜{azim[-1]:+.1f} (正が左)')
    print(f'  速さ  中央 {speeds[mid]:6.1f} m/s (理論 {THEORY_SPEED:.0f})  '
          f'破綻 {len(blown)} 発')

    # 中央と範囲だけだと「大半は揃っていて数発が暴れている」のか「全体に散って
    # いる」のかが区別できない。範囲は外れ弾 1 発に支配されるので、射出ごとの
    # 値を速さの順に並べて出す。射出輪の片側だけで蹴られた弾は速さが落ちて方位が
    # 散るので、この並びで見れば両者が対応しているかがそのまま読める。
    per_shot = sorted(
        (math.sqrt(vf ** 2 + vl ** 2 + vz ** 2),
         math.degrees(math.atan2(vl, vf)),
         math.degrees(math.atan2(vz, math.hypot(vf, vl))))
        for vf, vl, vz in vals)
    print('  射出ごと (速さの順):')
    for sp, az, el in per_shot:
        print(f'      {sp:6.1f} m/s   方位 {az:+7.1f}   仰角 {el:+6.1f}')
    if len(azim) >= 4:
        q1, q3 = azim[len(azim) // 4], azim[(3 * len(azim)) // 4]
        print(f'  方位の四分位  {q1:+.1f} 〜 {q3:+.1f} (幅 {q3 - q1:.1f} deg)')

    return {
        'flown': len(flown), 'backward': len(backward),
        'elev_med': elev[mid], 'elev_span': elev[-1] - elev[0],
        'azim_med': azim[mid], 'azim_span': azim[-1] - azim[0],
        'speed_med': speeds[mid], 'blown': len(blown), 'n': len(vals),
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot', default='sample_robot_1')
    ap.add_argument('--shots', type=int, default=12)
    ap.add_argument('--repeat', type=int, default=3,
                    help='同じ条件を何回まわすか。1 回では判断できない')
    ap.add_argument('--push', type=float, default=0.6)
    ap.add_argument('--retract', type=float, default=1.6)
    # 既定値は定数を直接書く。ここで WHEEL_VELOCITY を参照すると、
    # 後の global 宣言と衝突して SyntaxError になる。
    ap.add_argument('--wheel', type=float, default=500.0,
                    help='射出輪の指令速度 [rad/s]。250=10 m/s, 500=20, 800=32')
    args = ap.parse_args()

    global WHEEL_VELOCITY, THEORY_SPEED, BLOWUP_SPEED
    WHEEL_VELOCITY = args.wheel
    THEORY_SPEED = ROLLER_RADIUS * WHEEL_VELOCITY
    BLOWUP_SPEED = THEORY_SPEED * 1.6

    rclpy.init()
    node = Ballistics(args.robot)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    node.states.wait_for_service(timeout_sec=30.0)
    while node.pose is None:
        time.sleep(0.2)
    rx, ry, rz, yaw = node.pose
    print(f'射手 ({rx:+.2f},{ry:+.2f},{rz:+.3f}) yaw {math.degrees(yaw):+.1f} deg')
    print(f'射出口 {MUZZLE_Z:.3f} / 装甲板の帯 {ARMOR_LOW:.3f}〜{ARMOR_HIGH:.3f}')
    if rz > GROUNDED_Z:
        print(f'  !! 機体が接地していない (z={rz:.3f}、接地なら {GROUNDED_Z} 未満)。'
              f'構造物の上に乗っており、この状態では射出できない。'
              f'置き場所を変えて measure_hit.py と同じ手順で置き直すこと')
        rclpy.shutdown()
        return
    print()

    runs = []
    all_tracks = {}
    for i in range(args.repeat):
        tracks, launch = one_run(node, args.shots, args.push, args.retract)
        r = summarise(tracks, launch, args.shots, f'{i + 1} 回目')
        if r:
            runs.append(r)
        all_tracks.update(tracks)
        time.sleep(2.0)

    if len(runs) >= 2:
        print('\n=== 回ごとのばらつき ===')
        for key, name, unit in (('flown', '飛翔数', '発'),
                                ('speed_med', '速さの中央', 'm/s'),
                                ('elev_span', '仰角の幅', 'deg'),
                                ('azim_span', '方位の幅', 'deg'),
                                ('blown', '破綻', '発')):
            vals = [r[key] for r in runs]
            print(f'  {name:12s} ' + ' / '.join(f'{v:.1f}' for v in vals) +
                  f'  {unit}  (中央 {statistics.median(vals):.1f})')

    # 前方距離ごとの高さ。全回をまとめる。
    flown = {n: t for n, t in all_tracks.items()
             if t and max(abs(f) for f, _, _ in t) > 1.0}
    bins = {}
    for t in flown.values():
        for f, l, h in t:
            if f < 0:
                continue
            bins.setdefault(int(f), []).append((h, l))
    print('\n=== 前方距離ごとの高さと横ずれ (全回まとめ、飛翔中のみ) ===')
    print('   前方[m] 標本   高さ    横ずれ  装甲板の帯')
    in_band = []
    for d in sorted(bins):
        if d > 25:
            break
        hs = sorted(h for h, _ in bins[d])
        ls = sorted(l for _, l in bins[d])
        med = hs[len(hs) // 2]
        hit = ARMOR_LOW <= med <= ARMOR_HIGH
        if hit:
            in_band.append(d)
        print(f'    {d:5d} {len(hs):5d}  {med:+.3f}  {ls[len(ls) // 2]:+.3f}   '
              f'{"○" if hit else ""}')
    if in_band:
        print(f'\n  装甲板の高さに来る距離: {in_band}')
        print('  (連続した帯とは限らない。標本数の少ない距離は当てにしないこと)')
    else:
        print('\n  !! どの距離でも装甲板の高さに来ない')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
