import struct
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32, Float32MultiArray
import os
class PowerDataNode(Node):
    def __init__(self, index = 0):
        super().__init__('power_regress_node_' + str(index))
        self.X_accumulated = []
        self.P_accumulated = []
        self.index = index
        self.coff = np.array([0.02143351, 0.11251341, 0.18849983])
        self.intercept = np.array([1.9049197])

        self.msg = Float32()
        self.msg2 = Float32()
        self.msg_omegae = Float32()
        self.msg_current = Float32()
        self.msg_all = Float32MultiArray()
        # 创建订阅者和发布者
        self.subscription_omega = self.create_subscription(
            UInt8MultiArray,
            '/easy_robot_commands/dynamic/a' + str(index),  # 订阅主题名
            self.listener_callback,
            10
        )
        self.pub_power_pre = self.create_publisher(Float32, '/power_predict_' + str(index), 10)
        self.pub2 = self.create_publisher(Float32, '/power_real_' + str(index), 10)
        self.pub_omega = self.create_publisher(Float32, '/motor_omega_' + str(index), 10)
        self.pub_current = self.create_publisher(Float32, '/motor_current_' + str(index), 10)
        self.pub_all = self.create_publisher(Float32MultiArray, '/data_all_' + str(index), 10)

        self.get_logger().info('Power Regree Node initialized')

    
    def listener_callback(self, msg: UInt8MultiArray):
        # 确保接收到的字节数符合 Fdb_t 的大小
        if len(msg.data) != 24:
            print(f"Error: Expected 24 bytes, but got {len(msg.data)} bytes.")
            return
        # 定义结构体格式
        struct_format = '6f'  # 6 个单精度浮点数 (float)
        
        # 解包数据
        fdb_values = struct.unpack(struct_format, bytearray(msg.data))
        
        # 解析后的字段
        spe, cur, fdb_cur, power, omega, pre_power = fdb_values
        # 存储 omega cur power

        self.X_accumulated.append([np.sign(omega * cur) * omega * cur, 
                           omega * cur, cur * cur])
        self.P_accumulated.append(power)
        
        X_new = np.array([np.sign(omega * cur) * omega * cur, 
                           omega * cur, cur * cur])
        pre_p = self.coff @ X_new.transpose() + self.intercept
        self.msg.data = pre_p[0]
        self.pub_power_pre.publish(self.msg)

        self.msg2.data = power
        self.pub2.publish(self.msg2)
        
        self.msg_omegae.data = omega
        self.pub_omega.publish(self.msg_omegae)

        self.msg_current.data = cur
        self.pub_current.publish(self.msg_current)

        self.msg_all.data = [power, omega, cur, spe]
        self.pub_all.publish(self.msg_all)
        
        pass

    def save_data(self):
        np.save(os.path.dirname(__file__) +
                '/../data/X_accumulated_'+ str(self.index) + '.npy', np.array(self.X_accumulated))
        np.save(os.path.dirname(__file__) +
                '/../data/P_accumulated_'+ str(self.index) + '.npy', np.array(self.P_accumulated))
        self.get_logger().info("Data saved successfully!")



def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    indexes = [0, 1, 2, 3]
    nodes = [PowerDataNode(i) for i in indexes]
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        # 保存数据
        for node in nodes:
            node.save_data()
            pass
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
