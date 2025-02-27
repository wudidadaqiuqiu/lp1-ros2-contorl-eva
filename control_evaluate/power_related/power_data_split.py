import struct
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32, Float32MultiArray
import os
class PowerDataSplitNode(Node):
    def __init__(self):
        super().__init__('data_split_node')

        self.subscription_omega = self.create_subscription(
            UInt8MultiArray,
            '/easy_robot_commands/dynamic/a0',  # 订阅主题名
            self.listener_callback,
            10
        )
        self.pubs = [self.create_publisher(UInt8MultiArray, 
            '/easy_robot_commands/dynamic/a' + str(i), 10) for i in [1, 2, 3, 4]]

    def listener_callback(self, msg: UInt8MultiArray):
        # 确保接收到的字节数符合 Fdb_t 的大小
        if len(msg.data) != 24 * len(self.pubs):
            print(f"Error: Expected 24 bytes, but got {len(msg.data)} bytes.")
            return
        for i in range(4):
            part = msg.data[i * len(msg.data) // 4: (i + 1) * len(msg.data) // 4]
            new_msg = UInt8MultiArray()
            new_msg.data = part  # 设置切分后的数据
            self.pubs[i].publish(new_msg)  # 发布切分后的数据


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    nodes = [PowerDataSplitNode()]
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
