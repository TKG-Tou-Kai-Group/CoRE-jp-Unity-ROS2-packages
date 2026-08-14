"""`/load_world` を 1 回呼んで終了するノード。

Isaac 版の `isaac_ros2_scripts/launcher_with_reset` が USD ステージを開いていた
ところを、Unity 版ではシミュレータ本体は別プロセスで先に立ち上げておき、
このノードがワールド (SDF / シーン JSON) を読み込ませます。

`load_world` は既存のエンティティを全て消して STOPPED に戻すので、ロボットを
spawn する前に呼んでください。
"""

import os
import sys

import rclpy
from rclpy.node import Node

from simulation_interfaces.msg import Resource
from simulation_interfaces.srv import LoadWorld

# Result.msg: RESULT_OK = 1。規約に従っていないシミュレータが成功時に 0 を返す
# ことがあるので、simulation_ros2_utils と同じく両方を成功として受ける。
RESULT_OK_VALUES = (0, 1)


class LoadWorldNode(Node):

    def __init__(self):
        super().__init__('load_world')

        self.declare_parameter('world_path', '')
        world_path = self.get_parameter('world_path').get_parameter_value().string_value

        # 未対応要素やアセット欠けで読み込み全体を失敗にするか。既定は寛容側。
        # ステージのメッシュが 1 つ欠けただけでフィールドが出ない、という壊れ方を
        # 避けるため。
        self.declare_parameter('fail_on_unsupported_element', False)
        self.declare_parameter('ignore_missing_or_unsupported_assets', True)

        self.declare_parameter('service_timeout_sec', 120.0)
        timeout = self.get_parameter('service_timeout_sec').get_parameter_value().double_value

        self.ok = False

        if world_path == '':
            self.get_logger().error("parameter 'world_path' is empty")
            return
        if not os.path.isfile(world_path):
            self.get_logger().error(f'world file not found: {world_path}')
            return

        client = self.create_client(LoadWorld, 'load_world')
        waited = 0.0
        while not client.wait_for_service(timeout_sec=1.0):
            waited += 1.0
            if waited >= timeout:
                self.get_logger().error(
                    'load_world service did not appear; is the simulator and '
                    'ros_tcp_endpoint running?')
                return
            self.get_logger().info('load_world service not available, waiting...')

        request = LoadWorld.Request()
        request.world_resource = Resource()
        request.world_resource.uri = 'file://' + world_path
        request.world_resource.resource_string = ''
        request.fail_on_unsupported_element = \
            self.get_parameter('fail_on_unsupported_element').get_parameter_value().bool_value
        request.ignore_missing_or_unsupported_assets = \
            self.get_parameter(
                'ignore_missing_or_unsupported_assets').get_parameter_value().bool_value

        self.get_logger().info(f'loading world: {world_path}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            self.get_logger().error('load_world service call failed')
            return

        if response.result.result in RESULT_OK_VALUES:
            self.ok = True
            self.get_logger().info(f"world '{response.world.name}' loaded")
            # 読み飛ばした要素があると error_message に残るので、成功時も出す。
            if response.result.error_message:
                self.get_logger().warning(response.result.error_message)
        else:
            self.get_logger().error(
                f'load_world failed (result={response.result.result}): '
                f'{response.result.error_message}')


def main(args=None):
    rclpy.init(args=args)
    node = LoadWorldNode()
    ok = node.ok
    node.destroy_node()
    rclpy.shutdown()
    # launch 側で OnProcessExit を使って続きを繋げられるよう、失敗は終了コードで返す。
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
