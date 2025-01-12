import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32
from scipy.signal import butter, lfilter, lfilter_zi


class IIRFilterNode(Node):
    def __init__(self):
        super().__init__('iir_filter_node')

        # 创建订阅者和发布者
        self.subscription = self.create_subscription(
            Float32,
            'profileshow/a2',  # 订阅主题名
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(
            Float32,
            'filtered_data',  # 发布主题名
            10
        )

        # 初始化滤波器参数
        self.order = 10  # 滤波器阶数
        self.cutoff = 10.0  # 截止频率（Hz）
        self.fs = 500.0  # 采样频率（Hz）
        self.b, self.a = butter(self.order, self.cutoff / (self.fs / 2), btype='low')
        print(self.b)
        from scipy.signal import firwin
        self.coeff = firwin(30, self.cutoff / (self.fs / 2))  # 归一化截止频率

        # 滤波器的状态（适用于IIR滤波器）
        self.zi = lfilter_zi(self.b, self.a)

        self.get_logger().info('IIR Filter Node initialized')

    def listener_callback(self, msg: Float32):
        raw_data = msg.data

        # 如果是第一帧数据，初始化滤波器状态
        # if self.zi is None:
        #     self.zi = lfilter(self.b, self.a, [0], zi=lfilter(self.b, self.a, [0])[1])[1]

        # 对数据进行滤波
        filtered_data, self.zi = lfilter(self.b, self.a, [raw_data], zi=self.zi)

        # 发布滤波后的数据
        filtered_msg = Float32()
        filtered_msg.data = filtered_data[0]
        self.publisher.publish(filtered_msg)

        # 日志信息
        # self.get_logger().info(f'Received: {raw_data:.3f}, Filtered: {filtered_data[0]:.3f}')


def main(args=None):
    rclpy.init(args=args)

    node = IIRFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
