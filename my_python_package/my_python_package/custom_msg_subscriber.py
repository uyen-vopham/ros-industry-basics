import rclpy
from rclpy.node import Node
from simple_velocity_msg.msg import SimpleVelocity


class MySubscriber(Node):

    def __init__(self):
        super().__init__('simple_velocity_sub_node')

        self.subscriber_ = self.create_subscription(SimpleVelocity, '/simple_vel', self.subscriber_callback, 10)

        #------------------------------------------------
        #                    TODO:
        #  Create your subscriber below! Remember that your
        #  subscribes to SimpleVelocity type message from the 
        #  topic simple_vel
        #------------------------------------------------

    def subscriber_callback(self, msg):
        self.get_logger().info(f'Recieve linear velocity: "{msg.linear_velocity}" m/s, angular velocity: "{msg.angular_velocity}"')



    #------------------------------------------------
    #                    TODO:
    #  Create a callback function for your subscriber
    #  Inside this function create a line that logs
    #  the message

    #------------------------------------------------
        






def main(args=None):
    rclpy.init(args=args)

    my_subscriber = MySubscriber()

    rclpy.spin(my_subscriber)

    my_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()