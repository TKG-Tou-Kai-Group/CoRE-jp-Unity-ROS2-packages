#!/usr/bin/env python3
"""カメラのトピックを 1 枚受け取って PNG に落とすだけのノード。

sensor_msgs/Image と sensor_msgs/CompressedImage のどちらも受ける。
シミュレータは URDF の <format> によってどちらか一方しか出さないので、
--compressed で切り替える。

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
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class CaptureNode(Node):

    def __init__(self, topic, out_path, compressed):
        super().__init__('capture_image_topic')
        self.out_path = out_path
        self.bridge = CvBridge()
        self.saved = False
        # シミュレータ側は best effort で流してくるので合わせる
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        if compressed:
            self.create_subscription(CompressedImage, topic, self.on_compressed, qos)
        else:
            self.create_subscription(Image, topic, self.on_image, qos)
        self.get_logger().info(f'waiting for {topic}')

    def save(self, img, what):
        cv2.imwrite(self.out_path, img)
        self.get_logger().info(
            f'saved {self.out_path} ({img.shape[1]}x{img.shape[0]}, {what})')
        self.saved = True

    def on_image(self, msg):
        if self.saved:
            return
        self.save(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), msg.encoding)

    def on_compressed(self, msg):
        if self.saved:
            return
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn(f'{msg.format} を復号できなかった')
            return
        self.save(img, msg.format)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--compressed', action='store_true',
                    help='CompressedImage として受ける (URDF の format=jpeg のとき)')
    ap.add_argument('--timeout', type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    node = CaptureNode(args.topic, args.out, args.compressed)
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
