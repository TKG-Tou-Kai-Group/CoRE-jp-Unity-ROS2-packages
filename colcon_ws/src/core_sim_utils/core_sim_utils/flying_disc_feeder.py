"""フライングディスクを撃つたびに 1 枚ずつ供給し、全体で上限枚数に保つノード。

以前は起動時に 1 台あたり 20 枚を積み上げていた。積み荷は接触したまま積み重なった
剛体で、ソルバのコストが高い。実測ではロボット 2 台 + ディスク 40 枚で物理が実時間に
収まらなくなり、カメラ映像が 2.8 FPS まで落ちていた (同じ 40 枚でも、撃ち終えて
地面に散らばった状態なら 10.5 FPS 出る。静止した剛体はソルバから外れるため)。

そこで積み荷を持たず、装填口に常に 1 枚だけ置く。

  起動時            各機体の装填口へ 1 枚
  装填機構が引き切った 装填口が空なら次の 1 枚

引き戻しに「転じた瞬間」ではなく「引き切ってから」置くこと。ストローク 0.2 m を
1 m/s で戻るので 0.2 秒かかる一方、生成の往復は 100 ms しかない。転じた瞬間に
置くと、ローダーがまだ通路の中にいる間にディスクが出現して弾かれ、噛み方が
安定しない。

初速を付けて射出輪の間へ直接生成する方式は採れない。SpawnEntity に twist が無く、
生成してから set_entity_state で速度を与えると往復 2 回 (約 150 ms) の間に
自由落下で 11 cm 落ちてしまう。射出輪の幅は 3 cm しかないので帯から外れる。
底板に載せて装填機構で押す現状の経路をそのまま使うのが確実。

場に残るディスクは全機体あわせて max_alive 枚に保ち、超えたら古いものから消す。
機体ごとではなく全体で数えるのは、個別に管理する意味が薄いため。

delete_entity は積み荷のディスクに対して応答が返らなくなった前例があるので、
消す相手は「静止しているもの」に限り、呼び出しには必ず打ち切り時間を設ける。
"""

import collections
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from simulation_interfaces.msg import Resource, SpawnEntity as SpawnEntityMsg
from simulation_interfaces.srv import (
    DeleteEntity, GetEntitiesStates, SpawnEntities)

RESULT_OK_VALUES = (0, 1)

# 装填口へ置く位置 (base_link 基準)。
#
# x は通路の中央。通路は体軸 x が -0.10〜+0.12、y が ±0.1015 しかないので、
# 前方へずらすと射出輪に噛まないまま転がり出る (x を +0.05 にしたときは
# 28 ストロークで射出 0 発だった)。
#
# z は通路の中に直接置く。シュータの取り付けは shooter.urdf.xacro が射出口の
# 目標高さ (0.415) と仰角 (3.4 度) から逆算しており、base_link の +0.3512。
# 底板の上面がその +0.04、ディスクは厚み 0.02 なので中心は +0.4011 になる。
#
# 上から落とす方式は採れない。バレル (押さえ) が通路の上を覆っているので、
# それより高い位置から落とすと通路に入らず天井の上に乗る (以前 20 枚積み
# 方式の +0.55 をそのまま流用していて、飛翔 0 発になった)。
#
# shooter.urdf.xacro の shooter_mount_z を変えたらここも合わせること。
CHUTE_X = 0.0
CHUTE_Z = 0.4011
# 装填口に居るとみなす距離。これより遠ければ「空」と判断して補充する。
#
# 通路 (体軸 x が -0.10〜+0.12) と射出輪 (x = 0.15) を余裕をもって覆う大きさに
# してある。過検出は「補充しない」側に転ぶので、二重装填より安全。撃った弾は
# 20 m/s で出るので、この半径を抜けるのに 12 ms しかかからず、補充が遅れる
# 心配も無い。狭めるなら、通路の後ろで止まった弾を見落とさない値にすること。
CHUTE_RADIUS = 0.25
# 静止しているとみなす速度 [m/s]。飛翔中の弾を消さないための判定。
SETTLED_SPEED = 0.05
# 装填機構が引き切ったとみなす位置 [m]。ストロークは 0.2 m。
LOADER_HOME = 0.02
LOADER_JOINT = 'shooter_loader_link_joint'
# 引き切るのを待つ上限 [s]。0.2 m を 1 m/s で戻るので 0.2 s が目安。
LOADER_HOME_TIMEOUT = 2.0


class FlyingDiscFeeder(Node):

    def __init__(self):
        super().__init__('flying_disc_feeder')

        self.declare_parameter('urdf_path', '')
        self.urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value

        self.declare_parameter('robots', [f'sample_robot_{i}' for i in range(1, 9)])
        self.robots = list(
            self.get_parameter('robots').get_parameter_value().string_array_value)

        self.declare_parameter('max_alive', 24)
        self.max_alive = self.get_parameter('max_alive').get_parameter_value().integer_value

        self.declare_parameter('chute_x', CHUTE_X)
        self.declare_parameter('chute_z', CHUTE_Z)
        self.chute_x = self.get_parameter('chute_x').get_parameter_value().double_value
        self.chute_z = self.get_parameter('chute_z').get_parameter_value().double_value

        if not self.urdf_path or not os.path.isfile(self.urdf_path):
            self.get_logger().error(f"urdf_path が読めない: '{self.urdf_path}'")
            raise SystemExit(1)
        with open(self.urdf_path) as f:
            self.urdf = f.read()

        self.lock = threading.Lock()
        self.pose = {}          # robot -> (x, y, z, yaw)
        self.loader = {}        # robot -> 直近の装填指令
        self.alive = collections.deque()   # 生成順。FIFO の対象
        self.serial = 0
        self.busy = set()       # 補充処理中の機体 (二重生成を防ぐ)
        self.seeded = set()     # 一度でも装填できた機体 (タイマー経路から外す)
        self.loader_pos = {}    # robot -> 装填機構の位置 [m]

        for name in self.robots:
            self.create_subscription(
                PoseStamped, f'/{name}/ground_truth',
                lambda m, n=name: self.on_pose(n, m), 10)
            self.create_subscription(
                Float64MultiArray, f'/{name}/velocity_controller/commands',
                lambda m, n=name: self.on_command(n, m), 10)
            self.create_subscription(
                JointState, f'/{name}/joint_states',
                lambda m, n=name: self.on_joints(n, m), 10)

        self.spawner = self.create_client(SpawnEntities, 'spawn_entities')
        self.deleter = self.create_client(DeleteEntity, '/delete_entity')
        self.states = self.create_client(GetEntitiesStates, '/get_entities_states')

        # 生えてきた機体へ最初の 1 枚を入れる。以降の補充は引き戻しの合図
        # (on_command) だけが行う。
        #
        # 以前はこのタイマーが補充の 2 つ目の引き金として回り続けていた。
        # ストローク周期 (押し 0.6 s + 引き 1.6 s = 2.2 s) と 2 秒が近いため、
        # 位相がずれ続けて装填機構が通路に出ている瞬間にも発火する。実測では
        # 48 ストロークに対して 55〜58 枚が生成されていた。
        self.create_timer(2.0, self.top_up_all)
        self.get_logger().info(
            f'待機中: 機体 {len(self.robots)} 台 / 場のディスク上限 {self.max_alive} 枚')

    # ---- 受信 ---------------------------------------------------------

    def on_pose(self, robot, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        with self.lock:
            self.pose[robot] = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def on_joints(self, robot, msg):
        """装填機構の位置。引き切ったかの判定に使う。"""
        for name, position in zip(msg.name, msg.position):
            if name == LOADER_JOINT:
                with self.lock:
                    self.loader_pos[robot] = position
                return

    def wait_loader_home(self, robot):
        """装填機構が引き切るまで待つ。

        引き戻しに転じた「瞬間」に生成すると、ローダーがまだ通路の中に
        いる間に新しいディスクが出現して弾かれる。ストローク 0.2 m を
        1 m/s で戻るので 0.2 秒かかるが、生成の往復は 100 ms しかない。
        """
        end = time.time() + LOADER_HOME_TIMEOUT
        while time.time() < end:
            with self.lock:
                pos = self.loader_pos.get(robot)
            if pos is None or pos <= LOADER_HOME:
                return True
            time.sleep(0.02)
        return False

    def on_command(self, robot, msg):
        """装填機構の指令。押し出し (>0) から引き戻し (<=0) へ移った瞬間に補充する。"""
        if len(msg.data) < 3:
            return
        value = msg.data[2]
        with self.lock:
            previous = self.loader.get(robot, value)
            self.loader[robot] = value
        if previous > 0.0 >= value:
            threading.Thread(target=self.top_up, args=(robot,), daemon=True).start()

    # ---- サービス呼び出し ---------------------------------------------

    def _wait(self, future, timeout):
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.005)
        return future.done()

    @staticmethod
    def rotate(q, v):
        """クォータニオン q でベクトル v を回す。"""
        qx, qy, qz, qw = q
        tx = 2.0 * (qy * v[2] - qz * v[1])
        ty = 2.0 * (qz * v[0] - qx * v[2])
        tz = 2.0 * (qx * v[1] - qy * v[0])
        return (v[0] + qw * tx + (qy * tz - qz * ty),
                v[1] + qw * ty + (qz * tx - qx * tz),
                v[2] + qw * tz + (qx * ty - qy * tx))

    def chute_pose(self, robot):
        """装填口のワールド座標と、そこへ置くときの姿勢。

        機体の姿勢をそのまま使う (yaw だけでなく roll/pitch も)。装填口は
        機体に固定された場所なので、フィールドの台に乗って機体が持ち上がったり
        縁で傾いたりしても、装填口はそれに追従する。yaw しか見ないと、傾いた
        ときに通路の外へ置いてしまう。

        高さも機体の z からの相対で決まるので、台の上でもそのまま働く。
        """
        with self.lock:
            entry = self.pose.get(robot)
        if entry is None:
            return None
        (px, py, pz), q = entry
        ox, oy, oz = self.rotate(q, (self.chute_x, 0.0, self.chute_z))
        return (px + ox, py + oy, pz + oz, q)

    def disc_states(self, pattern, timeout=10.0):
        req = GetEntitiesStates.Request()
        req.filters.filter = pattern
        future = self.states.call_async(req)
        if not self._wait(future, timeout):
            return None
        res = future.result()
        if res is None:
            return None
        out = {}
        for name, st in zip(res.entities, res.states):
            p = st.pose.position
            v = st.twist.linear
            out[name] = ((p.x, p.y, p.z),
                         math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z))
        return out

    def spawn_one(self, robot, timeout=15.0):
        target = self.chute_pose(robot)
        if target is None:
            return False
        x, y, z, q = target

        self.serial += 1
        name = f'{robot}_flying_disc_{self.serial:04d}'

        req = SpawnEntities.Request()
        e = SpawnEntityMsg()
        e.name = name
        e.allow_renaming = False
        e.entity_resource = Resource()
        e.entity_resource.resource_string = self.urdf
        e.entity_namespace = ''
        e.initial_pose.header.frame_id = ''
        e.initial_pose.pose.position.x = x
        e.initial_pose.pose.position.y = y
        e.initial_pose.pose.position.z = z
        # 機体と同じ姿勢で置く。傾いた床に乗っていても、ディスクは装填口の
        # 底板と平行になる。
        e.initial_pose.pose.orientation.x = q[0]
        e.initial_pose.pose.orientation.y = q[1]
        e.initial_pose.pose.orientation.z = q[2]
        e.initial_pose.pose.orientation.w = q[3]
        req.spawn_requests.append(e)

        future = self.spawner.call_async(req)
        if not self._wait(future, timeout):
            self.get_logger().warning(f'{name}: 生成の応答なし')
            return False
        res = future.result()
        if res is None or res.result.result not in RESULT_OK_VALUES:
            msg = res.result.error_message if res else '応答なし'
            self.get_logger().warning(f'{name}: 生成に失敗 ({msg})')
            return False

        with self.lock:
            total = len(self.alive) + 1
            self.alive.append(name)
        self.get_logger().info(f'{name} を装填 (場に {total} 枚)')
        return True

    def delete_one(self, name, timeout=10.0):
        req = DeleteEntity.Request()
        req.entity = name
        future = self.deleter.call_async(req)
        if not self._wait(future, timeout):
            # 積み荷のディスクに対して応答が返らなくなった前例があるので、
            # 打ち切って次へ進む。取りこぼしても上限が少し緩むだけ。
            self.get_logger().warning(f'{name}: 削除の応答なし (打ち切り)')
            return False
        res = future.result()
        return res is not None and res.result.result in RESULT_OK_VALUES

    # ---- 供給と間引き ---------------------------------------------------

    def top_up_all(self):
        """まだ 1 枚も入っていない機体に最初の 1 枚を入れる。

        既に装填できた機体には触らない。以降の補充は引き戻しの合図が担う。
        撃った直後で装填口が空でも、次の引き戻しで必ず入るので取り残されない。
        """
        for robot in self.robots:
            with self.lock:
                if robot not in self.pose:
                    continue    # まだ生えていない機体は飛ばす
                if robot in self.seeded:
                    continue
            threading.Thread(target=self.top_up, args=(robot,), daemon=True).start()

    def top_up(self, robot):
        """装填口が空なら 1 枚入れる。空でなければ何もしない。

        成功時も 1 行ログを出す。無言にしていると、撃てないときに
        「補充が動いていないのか、補充はできていて射出側の問題なのか」を
        ログから切り分けられない。
        """
        with self.lock:
            if robot in self.busy:
                return
            self.busy.add(robot)
        try:
            # 装填機構が通路から抜けるまで待つ。抜ける前に置くと、ローダーの
            # 実体と重なった状態でディスクが出現し、貫入を解消する力積で後ろへ
            # 吹き飛ぶ。待ち切れなかったときは生成しない。以前は戻り値を見ずに
            # そのまま生成しており、余分な生成数と後ろへ飛んだ弾の数が実測で
            # ほぼ一致していた (48 ストロークで +7 枚 / 後ろへ 5 発)。
            if not self.wait_loader_home(robot):
                self.get_logger().warning(
                    f'{robot}: 装填機構が引き切らないので補充を見送る')
                return
            target = self.chute_pose(robot)
            if target is None:
                return
            discs = self.disc_states(f'^{robot}_flying_disc_[0-9]+$')
            if discs is None:
                return
            cx, cy, cz, _q = target
            for name, (pos, _speed) in discs.items():
                if math.dist(pos, (cx, cy, cz)) <= CHUTE_RADIUS:
                    self.get_logger().debug(
                        f'{robot}: 装填口に {name} が居るので補充しない')
                    return
            if self.spawn_one(robot):
                with self.lock:
                    self.seeded.add(robot)
                self.trim()
        finally:
            with self.lock:
                self.busy.discard(robot)

    def loaded_discs(self, states):
        """いずれかの機体の装填口に載っているディスクの名前。"""
        chutes = [c for c in (self.chute_pose(r) for r in self.robots) if c]
        held = set()
        for name, (pos, _speed) in states.items():
            for cx, cy, cz, _q in chutes:
                if math.dist(pos, (cx, cy, cz)) <= CHUTE_RADIUS:
                    held.add(name)
                    break
        return held

    def trim(self):
        """場のディスクが上限を超えたら、古いものから静止しているものを消す。

        上限に達するまでは 1 枚も消さない。撃った端から消えると的に届かない。
        装填口に載っている弾も消さない。撃つ前に弾が無くなるのを防ぐため。
        """
        with self.lock:
            excess = len(self.alive) - self.max_alive
            candidates = list(self.alive)[:max(excess, 0) + 8]
        if excess <= 0:
            return
        states = self.disc_states('.*flying_disc.*')
        if states is None:
            return
        held = self.loaded_discs(states)
        removed = 0
        for name in candidates:
            if removed >= excess:
                break
            if name in held:
                continue        # 装填口の弾は残す
            entry = states.get(name)
            if entry is None:
                # すでに居ない。台帳から外すだけ。
                with self.lock:
                    if name in self.alive:
                        self.alive.remove(name)
                continue
            _pos, speed = entry
            if speed > SETTLED_SPEED:
                continue        # 飛翔中は消さない
            if self.delete_one(name):
                with self.lock:
                    if name in self.alive:
                        self.alive.remove(name)
                removed += 1
        if removed:
            with self.lock:
                total = len(self.alive)
            self.get_logger().info(f'{removed} 枚を回収 (場に {total} 枚)')


def main(args=None):
    rclpy.init(args=args)
    node = FlyingDiscFeeder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
