#!/usr/bin/env python3

import sys
import numpy as np
import cv2
from math import cos, sin
import math, time
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import CompressedImage, Image
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerGuardCondition
from rclpy.utilities import timeout_sec_to_nsec

def wait_for_message(
    msg_type,
    node: 'Node',
    topic: str,
    time_to_wait=-1
):
    """
    Wait for the next incoming message.

    :param msg_type: message type
    :param node: node to initialize the subscription on
    :param topic: topic name to wait for message
    :param time_to_wait: seconds to wait before returning
    :returns: (True, msg) if a message was successfully received, (False, None) if message
        could not be obtained or shutdown was triggered asynchronously on the context.
    """
    context = node.context
    wait_set = _rclpy.WaitSet(1, 1, 0, 0, 0, 0, context.handle)
    wait_set.clear_entities()

    sub = node.create_subscription(msg_type, topic, lambda _: None, 1)
    try:
        wait_set.add_subscription(sub.handle)
        sigint_gc = SignalHandlerGuardCondition(context=context)
        wait_set.add_guard_condition(sigint_gc.handle)

        timeout_nsec = timeout_sec_to_nsec(time_to_wait)
        wait_set.wait(timeout_nsec)

        subs_ready = wait_set.get_ready_entities('subscription')
        guards_ready = wait_set.get_ready_entities('guard_condition')

        if guards_ready:
            if sigint_gc.handle.pointer in guards_ready:
                return False, None

        if subs_ready:
            if sub.handle.pointer in subs_ready:
                msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
                if msg_info is not None:
                    return True, msg_info[0]
    finally:
        node.destroy_subscription(sub)

    return False, None

def calculate_rectangle_area(coordinates):

    area = None
    width = None

    x1 = coordinates[0][0]
    y1 = coordinates[0][1]
    x2 = coordinates[1][0]
    y2 = coordinates[1][1]
    x3 = coordinates[2][0]
    y3 = coordinates[2][1]
    x4 = coordinates[3][0]
    y4 = coordinates[3][1]

    area = 0.5*((x1-x3)*(y2-y4) - (x2-x4)*(y1-y3))
    width = math.sqrt((x2-x1)**2 + (y2-y1)**2)

    return area, width

def detect_aruco(image):

    aruco_area_threshold = 1500
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])
    size_of_aruco_m = 0.15

    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []

    # Convert the BGR image to grayscale for aruco detection
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Define the aruco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # Detect aruco markers in the image and store 'corners' and 'ids'
    corners, ids, _ = detector.detectMarkers(gray)
    ids_threshold = []

    # Check if aruco markers are detected
    if ids is not None:
        for i in range(len(ids)):
            # Draw the detected marker on the image
            cv2.aruco.drawDetectedMarkers(image, corners, ids)

            # Calculate the area and width of the detected aruco marker
            coordinates = corners[i][0]
            area, width = calculate_rectangle_area(coordinates)

            # Remove markers that are far away from the arm's reach position based on the threshold
            if area > aruco_area_threshold:
                # Calculate center points of aruco markers
                center_x = np.mean(coordinates[:, 0])
                center_y = np.mean(coordinates[:, 1])
                center_aruco_list.append((center_x, center_y))

                # Estimate the pose of the aruco marker and calculate distance from the RGB camera
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], size_of_aruco_m, cam_mat, dist_mat)
                distance = np.linalg.norm(tvec)

                # Calculate the rotation matrix from the rotation vector (rvec)
                rot_mat, _ = cv2.Rodrigues(rvec)

                # Define the length of the axes (you can adjust this as needed)
                axis_length = 1  # Adjust this value to control the length of the axes

                # Calculate the endpoint positions of the axes in the marker's coordinate frame
                axis_points = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]])
                axis_points = np.array([np.dot(rot_mat, point.T) + tvec[0] for point in axis_points])

                # Convert the endpoints to pixel coordinates
                imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, cam_mat, dist_mat)

                # Draw the axes on the image
                imgpts = np.int32(imgpts).reshape(-1, 2)
                cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, axis_length)

                # Append the calculated values to their respective lists
                distance_from_rgb_list.append(distance)
                angle_aruco_list.append(rvec)
                width_aruco_list.append(width)
                ids_threshold.append(ids[i])

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids_threshold

class aruco_tf(Node):

    def __init__(self):

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        # self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        # self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())


    def depthimagecb(self, data):

        try:
            # Convert the ROS 2 Image message to a CV2 image
            self.depth_image = self.bridge.imgmsg_to_cv2(data)

        except Exception as e:
            self.get_logger().error("Error converting depth image: %s" % str(e))



    def colorimagecb(self, data):

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().error("Error converting color image: %s" % str(e))


    def process_image(self):
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375

        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids = detect_aruco(self.cv_image)
        marker_info_list = []

        # Loop over detected ArUco markers
        for i in enumerate(ids):
            i0 = i[0]
            marker_id = i[1][0]

            aruco_rvec = angle_aruco_list[i0]
            rot_mat, _ = cv2.Rodrigues(aruco_rvec)

            R = rot_mat

            sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
            
            singular = sy < 1e-6

            if  not singular :
                x = math.atan2(R[2,1] , R[2,2])
                y = math.atan2(-R[2,0], sy)
                z = math.atan2(R[1,0], R[0,0])
            else :
                x = math.atan2(-R[1,2], R[1,1])
                y = math.atan2(-R[2,0], sy)
                z = 0

            # Calculate quaternions from roll, pitch, and corrected yaw (yaw = angle_aruco)
            roll = math.pi/2
            pitch = 0.0
            yaw = math.pi/2 - z
            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            
            quaternion = [qx, qy, qz, qw]
            
            # Get depth from the RealSense camera (convert from mm to m)
            cX, cY = center_aruco_list[i0]
            depth = self.depth_image[int(cY)][int(cX)] / 1000.0

            # Calculate x, y, and z based on the camera parameters
            x = depth * (sizeCamX - cX - centerCamX) / focalX
            y = depth * (sizeCamY - cY - centerCamY) / focalY
            z = depth

            # Publish transform between camera_link and aruco marker center
            tf_camera_link_to_aruco = TransformStamped()
            tf_camera_link_to_aruco.header.stamp = self.get_clock().now().to_msg()
            tf_camera_link_to_aruco.header.frame_id = 'camera_link'
            tf_camera_link_to_aruco.child_frame_id = f'cam_{marker_id}'
            tf_camera_link_to_aruco.transform.translation.x = z
            tf_camera_link_to_aruco.transform.translation.y = x
            tf_camera_link_to_aruco.transform.translation.z = y
            self.br.sendTransform(tf_camera_link_to_aruco)  

            # Lookup transform between base_link and the aruco marker
            try:
                transform = self.tf_buffer.lookup_transform('base_link', f'cam_{marker_id}', rclpy.time.Time())
                # print('done')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.get_logger().error("Failed to lookup transform.")
                continue

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            position = [x, y, z]

            marker_list = [marker_id, position, quaternion]
            marker_info_list.append(marker_list)

        return marker_info_list


def main():
    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    # image = Image()
    image = wait_for_message(Image, node, '/camera/color/image_raw', 10)
    depth_image = wait_for_message(Image, node, '/camera/aligned_depth_to_color/image_raw', 10)

    aruco_tf_class.cv_image = aruco_tf_class.bridge.imgmsg_to_cv2(image[1], "bgr8")
    aruco_tf_class.depth_image = aruco_tf_class.bridge.imgmsg_to_cv2(depth_image[1])
    cv2.imshow('Image', aruco_tf_class.cv_image)
    cv2.waitKey(0)
    marker_list = aruco_tf_class.process_image()
    print(marker_list)


    print('done')
    # rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    main()
