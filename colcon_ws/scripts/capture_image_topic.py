#!/usr/bin/env python3
"""sensor_msgs/Image を 1 枚受け取って PNG に落とすだけのノード。

シミュレータの見た目を目視ではなく成果物で確かめたいとき用。ステージの向きや
マテリアルの当たり方は、ロボットの俯瞰カメラを保存すると一目で分かる。

    ros2 run --prefix "python3" ... ではなく直接実行する:
    python3 scripts/capture_image_topic.py \
        --topic /sample_robot_1/top_view_camera_link/image_raw \
        --out /tmp/top_view.png --timeout 60
"""

import argparse
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CaptureNode(Node):

    def __init__(self, topic, out_path):
        super().__init__('capture_image_topic')
        self.out_path = out_path
        self.bridge = CvBridge()
        self.saved = False
        # シミュレータ側は best effort で流してくるので合わせる
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, topic, self.on_image, qos)
        self.get_logger().info(f'waiting for {topic}')

    def on_image(self, msg):
        if self.saved:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(self.out_path, img)
        self.get_logger().info(
            f'saved {self.out_path} ({msg.width}x{msg.height}, {msg.encoding})')
        self.saved = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--timeout', type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    node = CaptureNode(args.topic, args.out)
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    while rclpy.ok() and not node.saved:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.get_clock().now().nanoseconds > deadline:
            node.get_logger().error('timed out waiting for an image')
            break
    ok = node.saved
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
