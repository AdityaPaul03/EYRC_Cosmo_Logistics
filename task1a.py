#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ CL#2736 ]
# Author List:		[ Ritesh Gole ]
# Filename:		    task1a.py
# Functions:
#			        [ calculate_rectangle_area, detect_aruco, depthimagecb, colorimagecb, process_image, main ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/color/image_raw, /camera/aligned_depth_to_color/image_raw ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image


##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.

    area = None
    width = None

    ############ ADD YOUR CODE HERE ############

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

    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################

    return area, width


def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15

    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
 
    ############ ADD YOUR CODE HERE ############

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

                # Calculate angle from the rotation vector (rvec)
                # angle = np.degrees(np.linalg.norm(rvec))
                angle = np.linalg.norm(rvec)

                # Append the calculated values to their respective lists
                distance_from_rgb_list.append(distance)
                angle_aruco_list.append(angle)
                width_aruco_list.append(width)
                ids_threshold.append(ids[i])
            
    # INSTRUCTIONS & HELP : 

    #	->  (Done) Convert input BGR image to GRAYSCALE for aruco detection         

    #   ->  (Done) Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)             

    #   ->  (Done) Detect aruco marker in the image and store 'corners' and 'ids'
    #       ->  HINT: Handle cases for empty markers detection.              

    #   ->  (Done) Draw detected marker on the image frame which will be shown later

    #   ->  (Done) Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))

    #   ->  (Done) Remove tags which are far away from arm's reach positon based on some threshold defined

    #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
    #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation

    #   ->  Draw frame axes from coordinates received using pose estimation
    #       ->  HINT: You may use 'cv2.drawFrameAxes'

    ############################################

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids_threshold


##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        try:
            # Convert the ROS 2 Image message to a CV2 image
            self.depth_image = self.bridge.imgmsg_to_cv2(data)

        except Exception as e:
            self.get_logger().error("Error converting depth image: %s" % str(e))

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().error("Error converting color image: %s" % str(e))

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################


    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375

        ############ ADD YOUR CODE HERE ############

        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids = detect_aruco(self.cv_image)

        # Loop over detected ArUco markers
        for i in enumerate(ids):
            i0 = i[0]
            marker_id = i[1][0]

            # Calculate the corrected angle
            angle_aruco = (0.788 * angle_aruco_list[i0]) - ((angle_aruco_list[i0] ** 2) / 3160)

            # Calculate quaternions from roll, pitch, and corrected yaw (yaw = angle_aruco)
            roll = 0.0
            pitch = 0.0
            yaw = angle_aruco
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

            # Mark the center point on the image frame
            cv2.circle(self.cv_image, (int(cX), int(cY)), 5, (0, 255, 0), -1)

            ''' Lafda starts'''

            # Publish transform between camera_link and aruco marker center
            tf_camera_link_to_aruco = TransformStamped()
            tf_camera_link_to_aruco.header.stamp = self.get_clock().now().to_msg()
            tf_camera_link_to_aruco.header.frame_id = 'camera_link'
            tf_camera_link_to_aruco.child_frame_id = f'cam_{marker_id}'
            tf_camera_link_to_aruco.transform.translation.x = z
            tf_camera_link_to_aruco.transform.translation.y = x
            tf_camera_link_to_aruco.transform.translation.z = y
            tf_camera_link_to_aruco.transform.rotation.x = quaternion[0]
            tf_camera_link_to_aruco.transform.rotation.y = quaternion[1]
            tf_camera_link_to_aruco.transform.rotation.z = quaternion[2]
            tf_camera_link_to_aruco.transform.rotation.w = quaternion[3]
            self.br.sendTransform(tf_camera_link_to_aruco)  

            # Lookup transform between base_link and the aruco marker
            try:
                transform = self.tf_buffer.lookup_transform('base_link', f'cam_{marker_id}', rclpy.time.Time())
                # print('done')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.get_logger().error("Failed to lookup transform.")
                continue

            # Publish transform between base_link and the aruco marker
            tf_base_link_to_obj = TransformStamped()
            tf_base_link_to_obj.header.stamp = self.get_clock().now().to_msg()
            tf_base_link_to_obj.header.frame_id = 'base_link'
            tf_base_link_to_obj.child_frame_id = f'obj_{marker_id}'
            tf_base_link_to_obj.transform = transform.transform
            self.br.sendTransform(tf_base_link_to_obj)

            ''' Lafda ends'''

        # Show the image with detected markers and center points
        cv2.imshow("Detected Markers", self.cv_image)
        cv2.waitKey(1)

        # INSTRUCTIONS & HELP : 

        #	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above

        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 

        #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
        #       It's a correction formula- 
        #       angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)

        #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)

        #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)

        #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
        #       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
        #       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
        #       z = distance_from_rgb
        #       where, 
        #               cX, and cY from 'center_aruco_list'
        #               distance_from_rgb is depth of object calculated in previous step
        #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

        #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 

        #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
        #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
        #       so that we will collect it's position w.r.t base_link in next step.
        #       Use the following frame_id-
        #           frame_id = 'camera_link'
        #           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, where 20 is aruco marker ID

        #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
        #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 

        #   ->  And now publish TF between object frame and base_link
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID

        #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/

        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1B)
        #               Also, auto eval script will be judging angular difference aswell. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()