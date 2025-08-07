import rclpy
from rclpy.node import Node, ReentrantCallbackGroup
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import qos_profile_sensor_data
from perception_msgs.srv import GetObjectLocation
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
from camera_service import transformations
import numpy as np
import cv2

class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        # Declare and read parameters [Aruco Marker Parameters & Camera Parameters]
        self.arcuo_detection = ArucoDetection()
        self.object_coordinates = None
        self.bridge = CvBridge()
        self.image_topic = '/image_raw'
        self.info_topic = '/camera_info'

        # Camera Parameters Initialization
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        server_cb_group = ReentrantCallbackGroup()
        sub_cb_group = server_cb_group

        self.camera_srv = self.create_service(GetObjectLocation, '/find_objects', self.service_callback, callback_group=server_cb_group)

        # Subscriptions - Camera Info & Image
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, qos_profile_sensor_data, callback_group = sub_cb_group)
        self.info_sub = self.create_subscription(CameraInfo, self.info_topic, self.info_callback, qos_profile_sensor_data)


    #------------------------------------------------
    #                    TODO:
    #  Populate the service callback here
    #------------------------------------------------
    def service_callback(self, request, response):
        print("you are in service_callback")
        if not self.object_coordinates:
            print("somthing happen")
            response.result = False
            response.object_pose = Pose()
            # Do Something if there is no tag detected
        else:
            print("nothing here")
            response.result = True
            response.object_pose = self.object_pose
            # pass
        return response
            # Do Something if there is a tag detected.

    # Marker Detection - Callback
    def image_callback(self, img_msg):
        gray_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        self.arcuo_detection.image_processing(gray_image, self.intrinsic_mat, self.distortion)
        self.object_coordinates = self.arcuo_detection.object_coordinates
        self.object_pose = self.arcuo_detection.pose
    
    # Camera Info - Callback
    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)



class ArucoDetection():
    def __init__(self):
        # Assign Parameters
        self.marker_size = 0.55
        dictionary_id_name = 'DICT_5X5_50'
        image_topic = '/image_raw'
        info_topic = '/camera_info'
        self.camera_frame = None
    
        # Dictionary ID Validation 
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_250):
                raise AttributeError
        except AttributeError:
            print(f'bad aruco_dictionary_id: {dictionary_id_name}')
            options = '\n'.join([s for s in dir(cv2.aruco) if s.startswith('DICT')])
            print(f'valid options: {options}')

        # Camera Parameters Initialization
        self.image_corners = None

        # Arcuo Parameters Initializatoin
        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.latest_image = None
        self.object_coordinates = None
        self.area = None
        self.pose = Pose()
        self.marker_ids = None


    def get_3d_pose(self, intrinsic_mat, distortion):
        if cv2.__version__ > '4.0.0':
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(self.corners, self.marker_size, intrinsic_mat, distortion)
        else:
            rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(self.corners, self.marker_size, intrinsic_mat, distortion)
        if(self.marker_ids is not None):
            for i, marker_id in enumerate(self.marker_ids):
                self.pose.position.x = tvecs[i][0][0]
                self.pose.position.y = tvecs[i][0][1]
                self.pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]

                quat = transformations.quaternion_from_matrix(rot_matrix)

                self.pose.orientation.x = quat[0]
                self.pose.orientation.y = quat[1]
                self.pose.orientation.z = quat[2]
                self.pose.orientation.w = quat[3]
                print(self.pose)


    # Marker Detection - Callback
    def image_processing(self, cv2_image, intrinsic_mat, distortion):
        self.latest_image = None
        self.object_coordinates = None
        self.area = None
        self.pose = Pose()
        self.marker_ids = None
        self.process_image(cv2_image, intrinsic_mat, distortion)
        self.display_image(self.latest_image)

    def process_image(self, cv2_image, intrinsic_mat, distortion):
        if self.latest_image is None:
            self.latest_image = np.zeros(cv2_image.shape, dtype=np.uint8)

        self.corners, self.marker_ids, rejected = cv2.aruco.detectMarkers(cv2_image, self.aruco_dictionary, parameters=self.aruco_parameters)
        self.get_3d_pose(intrinsic_mat, distortion)
        if len(self.corners) > 0:
            # print("Aruco marker detected!")
            self.latest_image = self.draw_aruco_bbox(cv2_image, self.corners, self.marker_ids)
            # self.object_coordinates = self.get_marker_coordinates()
        else:
            # print("No aruco marker detected!")
            self.latest_image = cv2_image
            self.object_coordinates = []

    def display_image(self, cv2_img):
        cv2.imshow("detected_box", cv2_img)
        if self.area is not None:
            print( "Area: " + str(self.area))
        # print("Number of objects: " + str(len(self.object_coordinates)))
        # for object_coord in self.object_coordinates:
        #     print(str(object_coord[0]) + "," + str(object_coord[1]))
        # if len(self.object_coordinates) != 0:
        #     print("\n\n")
        cv2.waitKey(1)

    def get_marker_coordinates(self):
        corners_ = self.corners.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners_
        return [((topLeft[0] + bottomRight[0])/2) , ((topLeft[1] + bottomRight[1])/2)]

    def draw_aruco_bbox(self,image,corners,ids):
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            # draw the bounding box of the Aruco detection
            cv2.line(image, topLeft, topRight, (0, 0, 255), 2)
            cv2.line(image, topRight, bottomRight, (0, 0, 255), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 0, 255), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 0, 255), 2)
            # compute and draw the center (x, y)-coordinates of the ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            # draw the Aruco marker ID on the image
            cv2.putText(image, str(markerID),
                (cX, cY + 30), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (0, 0, 255), 3)
            self.object_coordinates = [((topLeft[0] + bottomRight[0])/2),
                ((topLeft[1] + bottomRight[1])/2)]

            self.area = (abs(abs(topLeft[0]) - abs(bottomLeft[0]))) * (abs(abs(topLeft[0]) - abs(topRight[0])))
        # show the output image

        #return center of box
        return image

def main():
    rclpy.init()
    node = ArucoDetectionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
