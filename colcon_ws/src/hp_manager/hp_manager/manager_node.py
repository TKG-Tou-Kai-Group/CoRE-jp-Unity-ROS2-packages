import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
import threading

DAMAGE_PER_HIT = 10


class HPManager(Node):
    """装甲板の被弾を数えて HP を減らす。

    各装甲板は std_msgs/Int32 で「接触が始まった回数」を送ってくる。値は
    累積なので、購読側は 2 回の受信の差を取る。Bool の立ち上がりを見る方式
    から変えたのは、経路の途中でメッセージが捨てられても復帰できるため。

    シミュレータから ros_tcp_endpoint への経路は TCP 1 本 + Python 1 スレッド
    で、カメラを積むと実際に溢れて捨てられる。Bool だと捨てられた通に
    立ち上がりが乗っていた被弾は永久に失われるが、カウンタなら次に届いた
    1 通で差分としてまとめて入る。1 サンプルの間に 2 発当たった場合も、
    立ち上がりは 1 回しか出ないのに対し差分は 2 になる。
    """

    def __init__(self):
        super().__init__('hp_manager')
        self.declare_parameter('initial_hp', 100)
        self.initial_hp = self.get_parameter('initial_hp').get_parameter_value().integer_value
        self.declare_parameter('respawn_time_sec', 30.0)
        self.respawn_time_sec = self.get_parameter('respawn_time_sec').get_parameter_value().double_value

        self.hp = self.initial_hp
        self.hp_publisher = self.create_publisher(Int32, 'robot_hp', 10)

        # 装甲板ごとの前回値。まだ 1 通も受け取っていない板は None。
        self.last_counts = [None, None, None, None]
        for i in range(4):
            self.create_subscription(
                Int32, f'armor_topic_{i + 1}',
                lambda msg, i=i: self.armor_callback(i, msg), 10)

        # Subscriber for reset command
        self.reset_subscriber = self.create_subscription(Bool, '/reset_hp', self.reset_callback, 10)

        self.publish_hp_timer = self.create_timer(1.0, self.publish_hp)

    def armor_callback(self, index, msg):
        previous = self.last_counts[index]
        self.last_counts[index] = msg.data

        if previous is None:
            # 最初の 1 通は基準にするだけ。ここを 0 起点にすると、途中から
            # 購読を始めたときに過去の接触をまとめて食らうことになる。
            return

        hits = msg.data - previous
        if hits <= 0:
            # 減っていたらリセットか再スポーンでセンサが作り直された合図。
            # 負の差を被弾として扱わないよう、基準を取り直すだけにする。
            return

        self.apply_damage(hits)

    def apply_damage(self, hits):
        if self.hp <= 0:
            return

        self.hp = max(0, self.hp - DAMAGE_PER_HIT * hits)
        if self.hp == 0 and self.respawn_time_sec > 0:
            threading.Timer(self.respawn_time_sec, self.reset_hp).start()

    def reset_hp(self):
        self.hp = self.initial_hp

    def reset_callback(self, msg):
        if msg.data:
            self.reset_hp()

    def publish_hp(self):
        hp_msg = Int32()
        hp_msg.data = self.hp
        self.hp_publisher.publish(hp_msg)


def main(args=None):
    rclpy.init(args=args)
    hp_manager = HPManager()
    rclpy.spin(hp_manager)

    hp_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
