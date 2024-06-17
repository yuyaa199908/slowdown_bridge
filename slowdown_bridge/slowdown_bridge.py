import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

import array
import ctypes
import numpy as np
import struct
import time

class SlowdownBridge(Node):

    def __init__(self):
        super().__init__('slowdown_bridge')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.topic_a_callback,
            10
        )
        self.publisher = self.create_publisher(
            PointCloud2,
            '/livox/lidar_slow',
            0
        )

        self.publish_period = 0.33  # パブリッシュする周期
        self.do_filter = True
        self.filter_length = 3.5
        self.do_random_resample = True
        self.resample_to = 15000

    def topic_a_callback(self, msg):
        # パブリッシュ周期を確認してからトピックBにパブリッシュ
        now = time.time()
        if now - getattr(self, 'last_publish_time', 0) > self.publish_period:
            if self.do_filter:
                # width changes, height 1
                ret = array.array("B") # publish用に持っておく
                temp = []
                count = 0 # 距離フィルタ後の点群の点数
                for i in range(msg.height * msg.width):
                    d = msg.data[i*26:(i+1)*26]
                    (x, y, z) = struct.unpack("fff", d[0:12]) # decode to float value
                    dist = np.sqrt(x*x + y*y + z*z)
                    if dist <= self.filter_length:
                        temp.append(d) # if it remains, add to temporal list
                        # ret.extend(d)
                        count += 1
                if count > self.resample_to and self.do_random_resample:
                    # ランダム並び替え
                    idx_to_remain = np.random.default_rng().permutation([*range(len(temp))])
                    idx_to_remain = idx_to_remain[:self.resample_to]
                else:
                    idx_to_remain = [*range(count)] # if not downsampling, just use original length
                for j in idx_to_remain: # create data to return
                    ret.extend(temp[j])

            print(now, "filter from", msg.height * msg.width, "to", count, end="")
            if count > self.resample_to and self.do_random_resample:
                print(" after resample:", len(idx_to_remain))
            else:
                print("")

            msg.width = len(idx_to_remain)
            msg.data = ret
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
