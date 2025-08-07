import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose
from perception_msgs.srv import GetObjectLocation
import numpy as np
import time

class TurtleBotAruco(Node):
    def __init__(self):
        self.distance_threshold = 1.0
        self.max_linear_velocity = 0.2
        self.max_angular_velocity = 1.0
        self.scaled_linear_velocity = self.max_linear_velocity 
        self.scaled_angular_velocity = self.max_angular_velocity

        super().__init__('final_assessment')
        #------------------------------------------------
        #                    TODO:
        #  1) Create a ROS2 Publisher that sends Twist message to /cmd_vel 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        #------------------------------------------------
        #------------------------------------------------
        #                    TODO:
        #  2) Create a ROS2 Service Client of /find objects service, use type GetObjectLocation
        #------------------------------------------------
        
        self.service_client = self.create_client(GetObjectLocation, '/find_objects')
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        
    def move_turtlebot(self, linear_velocity, angular_velocity):
        self.twist_msg = Twist()
        self.twist_msg.linear.x = linear_velocity
        self.twist_msg.angular.z = angular_velocity
        
        self.publisher_.publish(self.twist_msg)
        #------------------------------------------------
        #                    TODO:
        #  3) Create a Twist Object, assign the linear and angular velocity
        #     to the correct parameter.
        #     Publish your Twist message to /cmd_vel
        #  Hint: Refer to your teleop_node (Practice 3!)
        #------------------------------------------------

    def check_tag(self):
        self.get_tag_pose()
        if self.response is not None:
            if self.response.result is False:
                self.stop_robot()
            else:
                self.handle_pose(self.response.object_pose)

    def get_tag_pose(self):
        #------------------------------------------------
        #                    TODO:
        #  Refer to Page 233 of your lecture slides!
        #  or you can refer to your object_spawner
        #  4) Create a getObjectLocation Request and assign it to self.req
        #  5) call the service client async function to send request, store future to self.future variable
        #------------------------------------------------
        self.req = GetObjectLocation.Request()
        self.future_ = self.service_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future_)

        
        if self.future_.result() is not None:
            self.response = self.future_.result()
            pose = self.response.object_pose
        else:
            self.get_logger().error('Service call failed')
    
    def stop_robot(self):
        self.move_turtlebot(0.0, 0.0)
    
    def get_tag_distance(self, x, y):
        return math.sqrt((x ** 2) + (y ** 2))

    def get_tag_angle(self, x, y):
        return np.arctan(y/x)

    def handle_pose(self, marker_pose):

        #------------------------------------------------
        #                    TODO:
        #  6) use self.get_tag_distance to get aruco tag distance from your camera!
        #  7) use self.get_tag_angle to get aruco tag angle from your camera!
        #------------------------------------------------
        tag_distance = self.get_tag_distance(marker_pose.position.z, marker_pose.position.x)
        tag_angle = self.get_tag_angle(marker_pose.position.z, marker_pose.position.x)

        # Given
        self.scaled_linear_velocity  = self.max_linear_velocity * (1 - (self.distance_threshold/tag_distance))
        self.scaled_angular_velocity = self.max_angular_velocity * (-tag_angle)
        self.move_turtlebot(self.scaled_linear_velocity, self.scaled_angular_velocity)

        # Logging
        self.get_logger().info(f'Tag Distance:{tag_distance}')
        self.get_logger().info(f'Tag Angle:{tag_angle}')
        self.get_logger().info(f'Angular Velocity:{self.scaled_angular_velocity}')
        self.get_logger().info(f'Linear Velocity:{self.scaled_linear_velocity}')

def main(args=None):
    rclpy.init(args=args)
    turtlebot_move = TurtleBotAruco()
    while(rclpy.ok()):
        turtlebot_move.check_tag()
        time.sleep(0.1)
    turtlebot_move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
