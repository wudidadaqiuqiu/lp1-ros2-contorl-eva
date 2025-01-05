import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from robot_msg.msg import MotorRef, NumReal, MotorFdb

class DataProcess(Node):
    def __init__(self, updated_data, mutex):
        super().__init__('data_process')
        self.log = self.get_logger()
        self.subs = {}
        self.subs['motor_ref'] = self.create_subscription(
            MotorRef,
            'motor6020_cmd',
            self.motor_callback,
            10
        )
        self.subs['motor_fdb'] = self.create_subscription(
            MotorFdb,
            'motor6020_fdb',
            self.motor_callback,
            10
        )
        self.name_map = { MotorFdb: 'motor_fdb', MotorRef: 'motor_ref'}

        self.updated_data: dict = updated_data
        self.mutex = mutex

    def get_field(self, msg):
        if type(msg) == MotorRef:
            return msg.pos_ref[0]
        elif type(msg) == MotorFdb:
            return msg.pos_zero_cross.deg.num

    def append_data(self, data: MotorRef | MotorFdb):
        key = self.name_map[type(data)]

        self.mutex.lock()
        self.updated_data[key] = self.updated_data[key][1:]  # 移除第一个点
        self.updated_data[key].append(self.get_field(data))  # 添加一个新的点
        self.mutex.unlock()

    def motor_callback(self, msg):
        self.append_data(msg)
    
def create_node(nodes, updated_data, mutex, node_attr):
    rclpy.init(args=None)
    nodes[node_attr] = DataProcess(updated_data, mutex)
    rclpy.spin(nodes[node_attr])
    nodes[node_attr].destroy_node()
    rclpy.shutdown()