import os
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32, Float32MultiArray

class PowerTryNode(Node):
    def __init__(self):
        super().__init__('power_try_node')
        self.X_accumulated = []
        self.P_accumulated = []
        self.omega_accumulated = []
        self.current_accumulated = []
        # self.coff = np.array([0.01723616, 0.32468812])
        self.coff = np.array([0.00479301, 0.01457816, 0.08997533])
        self.intercept = np.array([1.04431114])
        
        self.msgii = Float32()
        self.msgoi = Float32()
        self.msg_power = Float32()
        self.power = 0.0
        self.omega = 0.0
        self.current = 0.0

        self.subscription_power = self.create_subscription(
            Float32,
            '/power_real',  # 订阅主题名
            self.power_callback,
            10
        )
        # 创建订阅者和发布者
        self.subscription_omega = self.create_subscription(
            Float32,
            '/motor_omega',  # 订阅主题名
            self.omega_callback,
            10
        )
        self.sub_cur = self.create_subscription(
            Float32,
            '/motor_current',  # 订阅主题名
            self.current_callback,
            10
        )
        self.sub_all = self.create_subscription(
            Float32MultiArray,
            '/data_all',  # 订阅主题名
            self.all_data_callback,
            10
        )
        self.publisher_ii = self.create_publisher(Float32, '/power_ii', 10)
        self.publisher_oi = self.create_publisher(Float32, '/power_oi', 10)
        self.pub_es_power = self.create_publisher(Float32, '/es_power', 10)
    
    def power_callback(self, msg: Float32):
        self.power = msg.data

    def omega_callback(self, msg: Float32):
        self.omega = msg.data

    def all_data_callback(self, msg: Float32MultiArray):
        power, omega, current, spe = msg.data
        self.X_accumulated.append([np.sign(omega * current) * omega * current, 
                           omega * current, current * current])
        self.omega_accumulated.append(omega)
        self.current_accumulated.append(current)
        self.P_accumulated.append(power)

    def current_callback(self, msg: Float32):
        self.current = msg.data
        self.msgoi.data = self.omega * self.current * self.coff[1] + np.sign(self.omega * self.current) * self.omega * self.current * self.coff[0]
        self.publisher_oi.publish(self.msgoi)
        self.msgii.data = self.current * self.current * self.coff[2]
        self.publisher_ii.publish(self.msgii)

        # self.X_accumulated.append([np.sign(self.omega * self.current) * self.omega * self.current, 
        #                    self.omega * self.current, self.current * self.current])
        # self.P_accumulated.append(self.power)

        X_new = np.array([np.sign(self.omega * self.current) * self.omega * self.current, 
                          self.omega * self.current, self.current * self.current])
        pre_p = self.coff @ X_new.transpose() + self.intercept

        self.msg_power.data = pre_p[0]
        self.pub_es_power.publish(self.msg_power)
        

    def save_data(self):
        """ 保存数据到文件 """
        np.save(os.path.dirname(__file__) +'/../data/X_accumulated.npy', np.array(self.X_accumulated))
        np.save(os.path.dirname(__file__) +'/../data/P_accumulated.npy', np.array(self.P_accumulated))
        np.save(os.path.dirname(__file__) +'/../data/omega_accumulated.npy', np.array(self.omega_accumulated))
        np.save(os.path.dirname(__file__) +'/../data/current_accumulated.npy', np.array(self.current_accumulated))

        self.get_logger().info("Data saved successfully!")


def main(args=None):
    rclpy.init(args=args)

    node = PowerTryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 保存数据
        # print(__file__)
        node.save_data()
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
