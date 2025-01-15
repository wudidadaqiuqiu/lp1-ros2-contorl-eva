import subprocess
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

class ControlEvalSubscribe(Node):
    def __init__(self):
        super().__init__('control_eval_subscribe')
        self.subs = []
        # self.subscription  # prevent unused variable warning
    def add_subscribe(self, type_, topic_name, calback):
        self.subs.append((topic_name, self.create_subscription(
            type_,
            topic_name,
            calback,
            10
        )))


def node_init(args=None):
    rclpy.init(args=args)
    control_eval_subscribe = ControlEvalSubscribe()
    topic_checker_node = TopicCheckerNode()
    return control_eval_subscribe, topic_checker_node


def main(nodes):
    # rclpy.spin(node)
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    executor.spin()
    # Destroy the node explicitly
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()

class TopicCheckerNode(Node):
    def __init__(self):
        super().__init__('topic_checker_node')
        self.my_timer = self.create_timer(1.0, self.check_topics)
        self.topic_list = []
        # print("Topic Checker Node Initialized")

    def check_topics(self):
        # 获取当前所有话题
        result = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        topic_list = result.stdout.decode('utf-8').splitlines()
        self.topic_list = []
        for topic in topic_list:
            res = topic.split('/')
            res2 = []
            for a in res:
                if not a:
                    continue
                res2.append(a)
            self.topic_list.append(res2)
        # print(self.topic_list)

    def is_topic_starts_(self, topic_name):
        r = []
        for a in topic_name.split('/'):
            if not a:
                continue
            r.append(a)
        for topic in self.topic_list:
            b = True
            for i in range(len(topic)):
                if topic[i] != r[i]:
                    b = False
                    break
            if b:
                print(topic, topic_name)
                return '/'.join(topic)
        return None