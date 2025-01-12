import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Float32

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('mpc_publisher')  # 创建节点
        self.pub = self.create_publisher(Float32MultiArray, 'mpc_fdb', 10)  # 创建发布者，消息类型为 String
        self.sub = self.create_subscription(Float32, 'mpc_output', self.callback, 10)  # 创建订阅者，消息类型为 String
        self.param_pub = self.create_publisher(Float32MultiArray, 'mpc_param', 10)
        self.msg = Float32MultiArray()
        self.param_msg = Float32MultiArray()
        self.w, self.a = 0.0, 0.0
        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.timer_callback)  # 设置定时器回调函数
        self.timer1 = self.create_timer(1, self.timer_callback2)  # 设置定时器回调函数
        self.log = self.get_logger()
        self.log.info('mpc publisher started')

    def callback(self, msg: Float32):
        self.w += self.a * 0.001 + 0.5 * 0.001**2 * msg.data
        self.a += 0.001 * msg.data
        
    def timer_callback(self):
        # self.log.info('publishing: %s' % self.msg.data)
        self.msg.data = [self.w, self.a]
        self.pub.publish(self.msg)

    def timer_callback2(self):
        # load yaml
        with open('/home/xy/code/project/lp1-ros2-ws/src/control_evaluate/control_evaluate/config.yaml', 'r') as f:
            config = yaml.safe_load(f)
            # print(config)
            # print(self.param_msg.data)
            # print(config['param'])
            self.param_msg.data = config['param']
            self.param_pub.publish(self.param_msg)

def main(args=None):
    rclpy.init(args=args)  # 初始化ROS2
    minimal_publisher = MinimalPublisher()  # 创建发布者节点
    rclpy.spin(minimal_publisher)  # 节点运行

    # 销毁节点
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
