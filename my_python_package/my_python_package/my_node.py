import rclpy
import os
import select
import sys
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

INSTRUCTIONS = """
Control your TurtleBot3!
---------------------------
Moving around:
        w    
   a    s    d
        x    

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
s : stop

CTRL-C to quit
"""

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class MyExampleNode(Node):
    def __init__(self):
        super().__init__("my_example_node")
        self.counter = 0
        self.cmd_msg = Twist()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("my example node using OPP has started!")

        print(INSTRUCTIONS)

    def timer_callback(self):
        settings = termios.tcgetattr(sys.stdin) if os.name != 'nt' else None
        key = get_key(settings)

        if key == 'w':
            self.cmd_msg.linear.x += 0.1
            self.get_logger().info(f"Moving forward | Linear: {self.cmd_msg.linear.x:.2f} Angular: {self.cmd_msg.angular.z:.2f}")
        elif key == 'x':
            self.cmd_msg.linear.x -= 0.1
            self.get_logger().info(f"Moving backward | Linear: {self.cmd_msg.linear.x:.2f} Angular: {self.cmd_msg.angular.z:.2f}")
        elif key == 'a':
            self.cmd_msg.angular.z += 0.1
            self.get_logger().info(f"Turning left | Linear: {self.cmd_msg.linear.x:.2f} Angular: {self.cmd_msg.angular.z:.2f}")
        elif key == 'd':
            self.cmd_msg.angular.z -= 0.1
            self.get_logger().info(f"Turning right | Linear: {self.cmd_msg.linear.x:.2f} Angular: {self.cmd_msg.angular.z:.2f}")
        elif key == 's':
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.angular.z = 0.0
            self.get_logger().info("Stopping robot.")

        self.publisher_.publish(self.cmd_msg)
        

def main (args = None):
    rclpy.init(args=args)
    node = MyExampleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Exiting gracefully...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()