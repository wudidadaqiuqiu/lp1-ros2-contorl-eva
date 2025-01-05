import numpy as np
from scipy.signal import chirp
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from robot_msg.msg import MotorRef, NumReal


import matplotlib as mpl
def change_font():
    font_name = "simhei"
    mpl.rcParams['font.family']= font_name # 指定字体，实际上相当于修改 matplotlibrc 文件　只不过这样做是暂时的　下次失效
    mpl.rcParams['axes.unicode_minus']=False # 正确显示负号，防止变成方框

def signal_gen_test(fps, plot=False):
    # 定义参数
    t_start = 0  # 起始时间
    t_stop = 60  # 结束时间
    f_start = 0.2  # 起始频率
    f_stop = 10  # 结束频率

    # 生成时间向量
    t = np.linspace(t_start, t_stop, int((t_stop - t_start) * fps), endpoint=False)

    # 生成线性扫频信号
    signal = chirp(t, f_start, t_stop, f_stop, method='linear')

    # 绘制信号
    if plot:
        plt.figure()
        plt.plot(t, signal)
        plt.title('线性扫频信号')
        plt.xlabel('时间 (s)')
        plt.ylabel('幅度')
        plt.grid(True)
        plt.show()
    return signal


class SignalPublisher(Node):
    fps = 50  # 发布频率

    def __init__(self):
        super().__init__('signal_publisher')
        self.publisher_ = self.create_publisher(MotorRef, 'motor6020_cmd', 10)
        # self.timer_ = self.create_timer(1 / self.fps, self.publish_signal)  # 设定发布频率为100Hz
        self.cnt = 0
        self.sub = self.create_subscription(Float32MultiArray, 'signal_gen_cmd', self.trigger_callback, 10)
        self.log = self.get_logger()
        self.msg = MotorRef()
        self.msg.pos_ref = [NumReal()]

    def trigger_callback(self, msg):
        if self.cnt != 0:
            return
        self.log.info('trigger')
        self.timer_ = self.create_timer(1 / self.fps, self.publish_signal)  # 设定发布频率为100Hz
        self.signal = signal_gen_test(self.fps)

    def publish_signal(self):
        if self.cnt == len(self.signal):
            self.destroy_timer(self.timer_)  # 发布一次后销毁定时器
            self.cnt = 0
            return
        # self.log.info('publish')
        self.msg.pos_ref[0].num = self.signal[self.cnt] * 10
        self.cnt += 1
        self.publisher_.publish(self.msg)
        # self.get_logger().info('Publishing signal')


def main(args=None):
    rclpy.init(args=args)
    signal_publisher = SignalPublisher()
    rclpy.spin(signal_publisher)
    signal_publisher.destroy_node()
    rclpy.shutdown()

    # change_font()
    # signal_gen_test(100, True)

if __name__ == '__main__':
    main()
    