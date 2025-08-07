import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubriberNode (Node):
    def __init__(self):
        super().__init__("my_talker_node")
        self.counter = 0
        self.publisher_ = self.create_publisher(String, '/my_string',  10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        # self.get_logger().info('My subriber node has started and is listening to /my_string')

    def timer_callback (self):
        self.counter +=1
        msg = String()
        msg.data = f'Publishing message number {self.counter}'
        # self.get_logger().info(f'Received: {msg.data}')
        self.publisher_.publish((msg))

def main (args=None):
    rclpy.init(args=args)
    node = MySubriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ =='__main__':
    main()