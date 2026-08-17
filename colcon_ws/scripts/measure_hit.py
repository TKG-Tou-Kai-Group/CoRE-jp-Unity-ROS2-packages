#!/usr/bin/env python3
"""装甲板への被弾判定が取りこぼしていないかを測る。

    python3 scripts/measure_hit.py --shooter sample_robot_1 --target sample_robot_2

被弾は 2 つの経路で数えられる。

  真値   get_contact_events サービス。ContactReporter が OnCollisionEnter を
         物理ステップごとに拾って集計しているので、取りこぼしが無い
  実運用 /<機体名>/armorN_link/contact (std_msgs/Bool) の立ち上がりを
         hp_manager が数えて HP を 10 ずつ減らす

この 2 つを突き合わせる。Bool を出す ContactSensor は「publish 時点で接触して
いるか」を出すだけでラッチしないので、接触が publish 間隔より短いと落ちる。
ディスクは 32 m/s で厚さ 1 cm の装甲板を 0.3 ms で通過する一方、センサの
publish はフレームレート (既定 10 FPS) が上限なので、取りこぼしが出やすい。

射手と的は set_entity_state で配置する。競技の開始位置は互いに斜めを向いていて
当たらないため。

的は「角」を射手へ向ける。装甲板は機体の四隅 (体軸 ±0.35, ±0.35) に 45 度で
付いており、正面には無い。正対させると弾は相手のシュータ本体や車体に当たり、
装甲板には当たらない (実測: 正対で 4 m、装甲板への接触は真値でも 0 回)。

射出口の高さは shooter_barel_link (base_link の +0.54) ではなく射出輪
(shooter_base_link の +0.055 = base_link の +0.505)。装甲板は base_link の
+0.45 中心で 0.15 角なので帯は 0.375〜0.525、射出口はその中に入っている。
4 m なら落下は 0.08 m 程度なので、高さは問題にならない。
"""

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32, Float64MultiArray
from simulation_interfaces.srv import GetEntitiesStates, SetEntityState
from simulation_extra_interfaces.srv import GetContactEvents

# 射出輪の指令速度 [rad/s]。teleop の shoot_wheel_velocity と揃えること。
# 初速 = 射出輪の半径 0.04 x この値。500 で約 20 m/s。
WHEEL_VELOCITY = 500.0
LOADER_FEED = 1.0
LOADER_IDLE = -1.0
HP_PER_HIT = 10          # hp_manager の減少幅
RESULT_OK_VALUES = (0, 1)


# 装甲板の取り付け位置 (体軸)。板面は体軸から 45 度ずれた対角を向く。
ARMOR_OFFSETS = {
    'armor1_link': (0.35, 0.35),
    'armor2_link': (-0.35, 0.35),
    'armor3_link': (-0.35, -0.35),
    'armor4_link': (0.35, -0.35),
}
ARMOR_Z = 0.45
# 当たり得る横ずれの上限。ディスク半径 0.09 + 装甲板の半幅 0.075。
HIT_WIDTH = 0.165


def armor_world(pose, link):
    """機体の姿勢から装甲板のワールド位置を出す。"""
    x, y, _z, yaw = pose
    ax, ay = ARMOR_OFFSETS[link]
    return (x + ax * math.cos(yaw) - ay * math.sin(yaw),
            y + ax * math.sin(yaw) + ay * math.cos(yaw))


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class HitMeasure(Node):

    def __init__(s, shooter, target):
        super().__init__('measure_hit')
        s.lock = threading.Lock()
        s.hp = {}
        s.shooter = shooter
        s.target = target
        s.pose = {}
        for name in (shooter, target):
            s.create_subscription(
                Int32, f'/{name}/robot_hp',
                lambda m, n=name: s.on_hp(n, m), 10)
            s.create_subscription(
                PoseStamped, f'/{name}/ground_truth',
                lambda m, n=name: s.on_pose(n, m), 10)
        # 装甲板の接触トピックを直接見る。hp_manager と同じ立ち上がりを数えつつ、
        # 「一度でも True になったか」も別に持つ。HP が減らないとき、
        # センサが発火していないのか hp_manager 側の問題なのかを切り分けるため。
        s.armor_rise = {}
        s.armor_true = {}
        s._armor_prev = {}
        for i in range(1, 5):
            link = f'armor{i}_link'
            s.armor_rise[link] = 0
            s.armor_true[link] = 0
            s._armor_prev[link] = False
            s.create_subscription(
                Bool, f'/{target}/{link}/contact',
                lambda m, l=link: s.on_armor(l, m), 10)
        s.cmd = s.create_publisher(
            Float64MultiArray, f'/{shooter}/velocity_controller/commands', 10)
        s.setter = s.create_client(SetEntityState, '/set_entity_state')
        s.contacts = s.create_client(GetContactEvents, '/get_contact_events')
        s.states = s.create_client(GetEntitiesStates, '/get_entities_states')
        s.loader = LOADER_IDLE
        s.create_timer(0.1, s.publish_cmd)

    def publish_cmd(s):
        msg = Float64MultiArray()
        msg.data = [WHEEL_VELOCITY, WHEEL_VELOCITY, s.loader]
        s.cmd.publish(msg)

    def on_armor(s, link, msg):
        with s.lock:
            if msg.data:
                s.armor_true[link] += 1
                if not s._armor_prev[link]:
                    s.armor_rise[link] += 1
            s._armor_prev[link] = msg.data

    def on_hp(s, name, msg):
        with s.lock:
            s.hp[name] = msg.data

    def on_pose(s, name, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with s.lock:
            s.pose[name] = (p.x, p.y, p.z, yaw)

    def get_pose(s, name):
        with s.lock:
            return s.pose.get(name)

    def get_hp(s, name):
        with s.lock:
            return s.hp.get(name)

    def _wait(s, fut, timeout):
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            time.sleep(0.01)
        return fut.done()

    def place(s, entity, x, y, z, yaw, timeout=20.0):
        req = SetEntityState.Request()
        req.entity = entity
        req.set_pose = True
        req.set_twist = True
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        qx, qy, qz, qw = yaw_to_quat(yaw)
        req.state.pose.orientation.x = qx
        req.state.pose.orientation.y = qy
        req.state.pose.orientation.z = qz
        req.state.pose.orientation.w = qw
        fut = s.setter.call_async(req)
        if not s._wait(fut, timeout):
            return '応答なし'
        res = fut.result()
        if res is None or res.result.result not in RESULT_OK_VALUES:
            return f'result={res.result.result} {res.result.error_message}' if res else '応答なし'
        return None

    def discs(s, owner, timeout=25.0):
        """その機体のディスクの位置。射出されたかの判定に使う。"""
        req = GetEntitiesStates.Request()
        req.filters.filter = f'^{owner}_flying_disc_[0-9]+$'
        fut = s.states.call_async(req)
        if not s._wait(fut, timeout):
            return None
        res = fut.result()
        if res is None:
            return None
        return {n: (st.pose.position.x, st.pose.position.y, st.pose.position.z)
                for n, st in zip(res.entities, res.states)}

    def all_contacts(s, entity, clear=False, timeout=20.0):
        """その機体の全リンクの接触。{link: {相手: 回数}}"""
        req = GetContactEvents.Request()
        req.entity = entity
        req.clear = clear
        fut = s.contacts.call_async(req)
        if not s._wait(fut, timeout):
            return None
        res = fut.result()
        if res is None:
            return None
        out = {}
        for c in res.contacts:
            out.setdefault(c.link, {})
            out[c.link][c.other] = out[c.link].get(c.other, 0) + c.count
        return out

    def armor_contacts(s, entity, clear=False, timeout=20.0):
        """装甲板が受けた接触の真値。{link: (回数, 相手の集合)}"""
        req = GetContactEvents.Request()
        req.entity = entity
        req.clear = clear
        fut = s.contacts.call_async(req)
        if not s._wait(fut, timeout):
            return None
        res = fut.result()
        if res is None:
            return None
        out = {}
        for c in res.contacts:
            if not c.link.startswith('armor'):
                continue
            n, others = out.get(c.link, (0, set()))
            out[c.link] = (n + c.count, others | {c.other})
        return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--shooter', default='sample_robot_1')
    ap.add_argument('--target', default='sample_robot_2')
    ap.add_argument('--range', type=float, default=4.0, help='射手と的の距離 [m]')
    # 開始位置 (-4.5,-9.75) から +y へ 3 m は走行試験で通れることを確認済み。
    # 原点付近はフィールド構造物があり、置くと機体が浮いたり弾かれたりする。
    ap.add_argument('--x', type=float, default=-4.5, help='射手の x [m]')
    ap.add_argument('--y', type=float, default=-9.75, help='射手の y [m]')
    ap.add_argument('--face', choices=('corner', 'front'), default='corner',
                    help="的の向き。corner=角を射手へ向ける (装甲板が正対する)、"
                         "front=正面を向ける (装甲板は当たらない)")
    ap.add_argument('--shots', type=int, default=20, help='装填ストローク数')
    ap.add_argument('--push', type=float, default=1.5, help='押し出し [s]')
    ap.add_argument('--retract', type=float, default=1.5, help='引き戻し [s]')
    ap.add_argument('--spinup', type=float, default=4.0, help='射出輪の助走 [s]')
    ap.add_argument('--settle', type=float, default=5.0, help='最後の待ち [s]')
    args = ap.parse_args()

    rclpy.init()
    node = HitMeasure(args.shooter, args.target)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    for cli, name in ((node.setter, '/set_entity_state'),
                      (node.contacts, '/get_contact_events')):
        if not cli.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f'{name} が出ていない')

    # 手順を分ける。的を置いて落ち着かせ、装甲板が実際にどこへ来たかを測ってから、
    # そこへ射手を合わせる。摩擦 0 の接地脚なので置いた直後は滑って動き、
    # 狙った座標には止まらない (実測で 0.41 m ずれ、それで全弾外していた)。
    target_yaw = -math.pi / 2 - (math.pi / 4 if args.face == 'corner' else 0.0)
    for _ in range(2):
        e = node.place(args.target, args.x, args.y + args.range, 0.10, target_yaw)
        if e:
            raise RuntimeError(f'的の配置に失敗: {e}')
        time.sleep(6.0)

    pt = node.get_pose(args.target)
    if pt is None:
        raise RuntimeError('的の ground_truth が来ない')

    # 射手へ正対している装甲板 (的から見て -y 側にあるもの) を狙う
    aim_link = min(ARMOR_OFFSETS,
                   key=lambda k: armor_world(pt, k)[1])
    ax, ay = armor_world(pt, aim_link)

    # その装甲板の真南 range m に射手を置き、+y へ向ける
    for _ in range(2):
        e = node.place(args.shooter, ax, ay - args.range, 0.10, math.pi / 2)
        if e:
            raise RuntimeError(f'射手の配置に失敗: {e}')
        time.sleep(6.0)

    ps = node.get_pose(args.shooter)
    pt = node.get_pose(args.target)
    ax, ay = armor_world(pt, aim_link)
    offset = abs(ax - ps[0])          # 射線 (x 一定) から装甲板までの横ずれ
    dist = abs(ay - ps[1])
    print('配置の確認')
    print(f'  射手 ({ps[0]:+.2f},{ps[1]:+.2f},{ps[2]:+.3f}) yaw {math.degrees(ps[3]):+.1f} deg')
    print(f'  的   ({pt[0]:+.2f},{pt[1]:+.2f},{pt[2]:+.3f}) yaw {math.degrees(pt[3]):+.1f} deg')
    print(f'  狙う装甲板 {aim_link} ({ax:+.2f},{ay:+.2f})')
    print(f'  射線からの横ずれ {offset:.3f} m / 距離 {dist:.2f} m '
          f'(当たり得るのは横ずれ {HIT_WIDTH} m まで)')
    if offset > HIT_WIDTH:
        print('  !! 狙いが外れている。この結果は当たらなくて当然なので使えない')
    if ps[2] > 0.10 or pt[2] > 0.10:
        print('  !! 機体が浮いている (構造物の上)。置き場所を見直すこと')

    # ここまでの接触 (配置時の着地など) を捨てる
    node.all_contacts(args.target, clear=True)
    hp_before = node.get_hp(args.target)
    # 都度生成ではディスク名が毎回変わるので、事前後の名前突き合わせはしない。
    # 射出できているかは「的が受けた接触」と、生成された総数で見る。
    discs_before = node.discs(args.shooter) or {}

    node.loader = LOADER_IDLE
    time.sleep(args.spinup)

    for i in range(args.shots):
        node.loader = LOADER_FEED
        time.sleep(args.push)
        node.loader = LOADER_IDLE
        time.sleep(args.retract)
    time.sleep(args.settle)

    truth = node.armor_contacts(args.target) or {}
    every = node.all_contacts(args.target) or {}
    discs_after = node.discs(args.shooter) or {}
    fresh = set(discs_after) - set(discs_before)
    print()
    print(f'  試験中に補充されたディスク {len(fresh)} 枚 '
          f'(都度生成なので、撃った回数の目安になる)')
    if every:
        print('  的が受けた接触 (全リンク):')
        for link in sorted(every):
            for other, n in sorted(every[link].items()):
                print(f'      {link:18s} <- {other:28s} {n} 回')
    else:
        print('  的は何にも触れていない (弾が届いていない)')
    if discs_after:
        near = [(n, math.dist(p, (pt[0], pt[1], p[2])))
                for n, p in discs_after.items()]
        near.sort(key=lambda kv: kv[1])
        print('  的に最も近づいたディスクの最終位置 (上位3件):')
        for n, d in near[:3]:
            p = discs_after[n]
            print(f'      {n} ({p[0]:+.2f},{p[1]:+.2f},{p[2]:+.2f})  的から {d:.2f} m')
    hp_after = node.get_hp(args.target)
    pt2 = node.get_pose(args.target)
    moved = math.hypot(pt2[0] - pt[0], pt2[1] - pt[1]) if pt2 else float('nan')
    print()
    print(f'  的の移動量 {moved:.2f} m (撃たれて押された量。'
          f'大きいと途中から的を外している)')

    with node.lock:
        rise = dict(node.armor_rise)
        true_n = dict(node.armor_true)
    print('  装甲板の接触トピック (hp_manager が見ているもの):')
    for link in sorted(rise):
        print(f'      {link}: True を {true_n[link]} 回受信 / '
              f'立ち上がり {rise[link]} 回')
    total_rise = sum(rise.values())

    truth_total = sum(n for n, _ in truth.values())
    print(f'射手 {args.shooter} -> 的 {args.target} / 距離 {args.range} m / '
          f'{args.shots} ストローク')
    print(f'  真値の被弾数 (get_contact_events) {truth_total}')
    for link in sorted(truth):
        n, others = truth[link]
        print(f'      {link}: {n} 回  相手={sorted(others)}')
    if hp_before is None or hp_after is None:
        print('  HP        取得できない (robot_hp が来ていない)')
        rclpy.shutdown()
        return
    detected = (hp_before - hp_after) // HP_PER_HIT
    print(f'  HP        {hp_before} -> {hp_after}  '
          f'= 判定された被弾 {detected} 発')
    if total_rise > 0:
        print(f'  センサの立ち上がり {total_rise} 回 -> HP から見た被弾 {detected} 発')
        if detected < total_rise:
            print(f'  !! hp_manager が {total_rise - detected} 回取りこぼしている')
    else:
        print('  装甲板のセンサが一度も発火していない。'
              '当たっていないか、センサが拾えていないかのどちらか')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
