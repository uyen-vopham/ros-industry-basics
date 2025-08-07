import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubriberNode (Node):
    def __init__(self):
        super().__init__("my_listener_node")
        self.subscription_ = self.create_subscription(String, '/my_string', self.listener_callback, 10)
        self.get_logger().info('My subriber node has started and is listening to /my_string')

    def listener_callback (self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main (args=None):
    rclpy.init(args=args)
    node = MySubriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ =='__main__':
    main()