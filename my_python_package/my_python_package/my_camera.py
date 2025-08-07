import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2

class MyViewCamera (Node):
    def __init__(self):
        super().__init__('my_camera_node')

        self.image_ = Image()
        self.subscription_ = self.create_subscription(Image, '/image_raw', self.subscription_callback, 10)
        self.bridge = CvBridge()
        self.image_window = cv2.namedWindow("Camera Output")

    def subscription_callback(self, msg):
        self.view_image(msg)

    def view_image(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        cv2.imshow("image", cv_image)
        cv2.waitKey(1)

def main (args=None):
    rclpy.init(args=args)
    node = MyViewCamera()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ =='__main__':
    main()
