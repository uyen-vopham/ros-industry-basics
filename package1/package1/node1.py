import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Node1Publisher(Node):
    def __init__(self):
        super().__init__('node1_publisher')
        self.publisher = self.create_publisher(String, '/my_string', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # publish every 1 second
        self.count = 0
        self.get_logger().info('node1_publisher has started and is publishing to /my_string')

    def publish_message(self):
        msg = String()
        msg.data = f'Hello from Node1! Count: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = Node1Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
