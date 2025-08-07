
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Pose
from perception_msgs.srv import GetObjectLocation
import rclpy
from rclpy.node import Node

class ObjectSpawnerNode(Node):
    def __init__(self):
        super().__init__('object_spawner_node')

        self.publish_marker_ = self.create_publisher(Marker, '/rviz_markers', 10)
        self.client_ = self.create_client(GetObjectLocation, '/find_objects')

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service is not available, waiting again ...')

        


    def send_request(self):

    #------------------------------------------------
    #                    TODO:

    #  Populate the send_request function.This is where you should craft your 
    #  request and read the response to be passed to the publish_markers function
    #  In the publish_markers fuction do the following
    #------------------------------------------------
    #step1: create request
        self.req = GetObjectLocation.Request()
    #step2: send request 
        self.future = self.client_.call_async(self.req)
    #step3: spin until you revieve a response from server
        rclpy.spin_until_future_complete(self, self.future)
    #step4: once you received the response, get the result
        self.response = self.future.result()
        # self.get_logger().info(f"Pose position is {self.response.object_pose.position}")
        # self.get_logger().info(f"Pose angular is {self.response.object_pose.orientation}")
        self.publish_marker(self.response.object_pose)
        

    def publish_marker(self, pose):		
        marker = Marker()
        marker.pose = pose

        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 1
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 5.0
        marker.scale.y = 5.0
        marker.scale.z = 5.0

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.publish_marker_.publish(marker)


def main():
    rclpy.init()
    node = ObjectSpawnerNode()
    while rclpy.ok():
        node.send_request()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
