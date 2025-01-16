import struct
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32, Float32MultiArray
from sklearn.linear_model import SGDRegressor

# model = SGDRegressor(
#     loss='huber',  # 损失函数类型
#     penalty='l2',  # 正则化类型
#     alpha=0.8,  # 正则化强度
#     fit_intercept=True,  # 是否计算截距
#     max_iter=50000,  # 最大迭代次数
#     tol=0.01,  # 收敛容忍度
#     shuffle=True,  # 是否打乱数据
#     verbose=0,  # 输出的详细程度
#     random_state=None,  # 随机数生成器的种子
#     learning_rate='optimal',  # 学习率类型
#     epsilon=1,
#     eta0=0.01,  # 初始学习率
#     power_t=0.25,  # 学习率调整的幂
#     early_stopping=False,  # 是否启用早期停止
#     validation_fraction=0.1,  # 验证集的比例
#     n_iter_no_change=10,  # 连续多少轮未改进则停止
#     warm_start=False,  # 是否使用温启动
#     average=False  # 是否使用平均梯度
# )

class PowerRegressNode(Node):
    def __init__(self):
        super().__init__('power_regress_node')
        self.X_accumulated = []
        self.P_accumulated = []
        self.coff = np.array([0.01723616, 0.32468812])
        # self.coff = np.array([0.00455999, 0.01510337, 0.10810623])
        self.intercept = np.array([1.04431114])

        self.msg = Float32()
        self.msg2 = Float32()
        self.msg_omegae = Float32()
        self.msg_current = Float32()
        self.msg_all = Float32MultiArray()
        # 创建订阅者和发布者
        self.subscription_omega = self.create_subscription(
            UInt8MultiArray,
            '/easy_robot_commands/dynamic/a2',  # 订阅主题名
            self.listener_callback,
            10
        )
        self.pub = self.create_publisher(Float32, '/power_predict', 10)
        self.pub2 = self.create_publisher(Float32, '/power_real', 10)
        self.pub_omega = self.create_publisher(Float32, '/motor_omega', 10)
        self.pub_current = self.create_publisher(Float32, '/motor_current', 10)
        self.pub_all = self.create_publisher(Float32MultiArray, '/data_all', 10)

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
        
        # print(f"Parsed Fdb_t values:")
        # print(f"  spe: {spe}")
        # print(f"  cur: {cur}")
        # print(f"  fdb_cur: {fdb_cur}")
        # print(f"  power: {power}")
        # print(f"  omega: {omega}")
        # print(f"  pre_power: {pre_power}")

        X1_new = omega * cur
        X2_new = cur ** 2
        
        X_new = np.array([[X1_new, X2_new]])
        pre_p = self.coff @ X_new.transpose() + self.intercept

        self.msg.data = pre_p[0]
        self.pub.publish(self.msg)

        self.msg2.data = power
        self.pub2.publish(self.msg2)
        
        self.msg_omegae.data = omega
        self.pub_omega.publish(self.msg_omegae)

        self.msg_current.data = cur
        self.pub_current.publish(self.msg_current)

        self.msg_all.data = [power, omega, cur]
        self.pub_all.publish(self.msg_all)
        
        # P_new = np.array([power])
        self.X_accumulated.append([X1_new, X2_new])
        self.P_accumulated.append(power)
        
        # if abs(X1_new) > 0.1 and abs(X2_new) > 0.1:
        #     model.partial_fit(X_new, np.array([power]))
        #     self.coff = model.coef_
        #     self.intercept = model.intercept_

        # model.partial_fit(X_new, P_new)
        pass

    def save_data(self):
        """ 保存数据到文件 """
        np.save('data/X_accumulated.npy', np.array(self.X_accumulated))
        np.save('data/P_accumulated.npy', np.array(self.P_accumulated))
        self.get_logger().info("Data saved successfully!")


def main(args=None):
    rclpy.init(args=args)

    node = PowerRegressNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 保存数据
        # node.save_data()
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
