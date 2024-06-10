import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

import time
import numpy as np
import ctypes

class SlowdownBridge(Node):

    def __init__(self):
        super().__init__('slowdown_bridge')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.topic_a_callback,
            10
        )
        self.publisher = self.create_publisher(
            PointCloud2,
            '/velodyne_points_slow',
            0
        )

        self.publish_period = 1.0  # パブリッシュする周期（2 Hz）

    def topic_a_callback(self, msg):
        # パブリッシュ周期を確認してからトピックBにパブリッシュ
        now = time.time()
        if now - getattr(self, 'last_publish_time', 0) > self.publish_period:
            self.publisher.publish(msg)

            self.last_publish_time = now
            
def main(args=None):
    rclpy.init(args=args)
    topic_a_bridge = SlowdownBridge()
    rclpy.spin(topic_a_bridge)
    topic_a_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()