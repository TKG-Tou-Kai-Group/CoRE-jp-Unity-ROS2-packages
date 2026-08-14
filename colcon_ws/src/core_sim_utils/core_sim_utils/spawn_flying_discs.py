"""フライングディスクを積み上げてスポーンするノード。

Isaac 版はディスクを N 枚並べた USD (`flying_disc_20set.usd`) を `add_usd` で
1 回置いていました。Unity 版には「複数剛体を含むプレハブ」に相当する入口が
無いので、1 枚ぶんの URDF を `spawn_entities` で N 個まとめて要求します。

姿勢はロボットのシュータ位置に合わせて渡します。ディスクは `z_spacing` ずつ
積み上がるので、ロボットの装填口の高さから上へ積む使い方を想定しています。
"""

import math
import os
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from simulation_interfaces.msg import Resource, SpawnEntity as SpawnEntityMsg
from simulation_interfaces.srv import SpawnEntities

RESULT_OK_VALUES = (0, 1)


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class SpawnFlyingDiscsNode(Node):

    def __init__(self):
        super().__init__('spawn_flying_discs')

        self.declare_parameter('urdf_path', '')
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value

        # エンティティ名の接頭辞。ロボットごとに変えないと 2 台目以降が
        # 名前衝突する (allow_renaming を立てていないので失敗になる)。
        self.declare_parameter('name_prefix', 'flying_disc')
        name_prefix = self.get_parameter('name_prefix').get_parameter_value().string_value

        self.declare_parameter('count', 20)
        count = self.get_parameter('count').get_parameter_value().integer_value

        for name, default in (('x', 0.0), ('y', 0.0), ('z', 0.0),
                              ('R', 0.0), ('P', 0.0), ('Y', 0.0),
                              ('z_spacing', 0.025)):
            self.declare_parameter(name, default)

        def p(name):
            return self.get_parameter(name).get_parameter_value().double_value

        self.declare_parameter('service_timeout_sec', 120.0)
        timeout = self.get_parameter('service_timeout_sec').get_parameter_value().double_value

        self.ok = False

        if urdf_path == '' or not os.path.isfile(urdf_path):
            self.get_logger().error(f"urdf_path not found: '{urdf_path}'")
            return
        if count <= 0:
            self.get_logger().warning('count <= 0, nothing to spawn')
            self.ok = True
            return

        with open(urdf_path, 'r') as f:
            urdf_content = f.read()

        client = self.create_client(SpawnEntities, 'spawn_entities')
        waited = 0.0
        while not client.wait_for_service(timeout_sec=1.0):
            waited += 1.0
            if waited >= timeout:
                self.get_logger().error('spawn_entities service did not appear')
                return
            self.get_logger().info('spawn_entities service not available, waiting...')

        qx, qy, qz, qw = euler_to_quaternion(p('R'), p('P'), p('Y'))

        request = SpawnEntities.Request()
        for i in range(count):
            spawn = SpawnEntityMsg()
            spawn.name = f'{name_prefix}_{i:02d}'
            spawn.allow_renaming = False
            spawn.entity_resource = Resource()
            spawn.entity_resource.uri = 'file://' + urdf_path
            spawn.entity_resource.resource_string = urdf_content
            # ディスクはトピックを持たないので namespace は不要。
            spawn.entity_namespace = ''
            spawn.initial_pose.header.frame_id = ''
            spawn.initial_pose.pose.position.x = p('x')
            spawn.initial_pose.pose.position.y = p('y')
            spawn.initial_pose.pose.position.z = p('z') + i * p('z_spacing')
            spawn.initial_pose.pose.orientation.x = qx
            spawn.initial_pose.pose.orientation.y = qy
            spawn.initial_pose.pose.orientation.z = qz
            spawn.initial_pose.pose.orientation.w = qw
            request.spawn_requests.append(spawn)

        self.get_logger().info(f"spawning {count} discs as '{name_prefix}_NN'")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            self.get_logger().error('spawn_entities service call failed')
            return

        if response.result.result in RESULT_OK_VALUES:
            self.ok = True
            self.get_logger().info(f'{count} flying discs spawned')
        else:
            # ENTITIES_SPAWN_FAILED (150) のときは個別の結果に理由が入る。
            self.get_logger().error(
                f'spawn_entities failed (result={response.result.result}): '
                f'{response.result.error_message}')
            for i, r in enumerate(response.results):
                if r.result.result not in RESULT_OK_VALUES:
                    self.get_logger().error(f'  [{i}] {r.result.error_message}')


def main(args=None):
    rclpy.init(args=args)
    node = SpawnFlyingDiscsNode()
    ok = node.ok
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
