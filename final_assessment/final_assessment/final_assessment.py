import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose
from perception_msgs.srv import GetObjectLocation
import numpy as np
import time

class TurtleBotAruco(Node):
    def __init__(self):
        super().__init__('final_assessment')
        # Recommended values, change if you want!
        self.distance_threshold = 3.0
        self.max_linear_velocity = 0.1
        self.max_angular_velocity = 1.0
	#create client to /find_objects service
        self.client_ = self.create_client(GetObjectLocation, '/find_objects')
        
        #create publisher to /cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #Inherit object
        self.msg_pub_ = Twist()	
        self.pose_ = Pose()
        
        #Other variables
        self.moving_ = True
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_angular = 0.0

    def check_tag(self):
        #-----------------------------------------------------------------------
        #                    TODO:
        #  Create a function check the tag poses, and move the robot accordingly
        #------------------------------------------------------------------------
        self.req = GetObjectLocation.Request()
        self.future = self.client_.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.response = self.future.result()
        self.x_pos = self.response.object_pose.position.x
        self.y_pos = self.response.object_pose.position.y
        self.z_angular = self.response.object_pose.orientation.z
        if (self.reponse.result == True) and (self.moving_ == True):
            self.move_turtlebot(self.max_linear_velocity, 0.0)
        else:
            self.move_turtlebot(0.0, 0.0)
        
        if (self.get_tag_distance(self.x_pos, self.y_pos) == self.distance_threshold):
            self.moving_ = False   
    
    def get_tag_pose(self):
        if (self.response.result):
            self.pose_ = self.response.object_pose #recieve position and orientation
        else:
            print("get_Tag_fucntion")
            # return
        	

    def move_turtlebot(self, linear_velocity, angular_velocity):
        #------------------------------------------------
        #                    TODO:
        #  Create a function to move the robot
        #------------------------------------------------
        self.msg_pub_.linear.x = linear_velocity
        self.msg_pub_.angular.z = angular_velocity
        self.publisher_.publisth(self.msg_pub_)

    def get_tag_distance(self, x, y):
        return math.sqrt((x ** 2) + (y ** 2))

    def get_tag_angle(self, x, y):
        return np.arctan(y/x)

def main(args=None):
    rclpy.init(args=args)
    turtlebot_move = TurtleBotAruco()
    while(rclpy.ok()):
        turtlebot_move.check_tag()
    turtlebot_move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()