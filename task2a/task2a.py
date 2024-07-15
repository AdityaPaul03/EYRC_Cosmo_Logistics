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
# Filename:		    task1b.py
# Functions:
#			        [ main ]
# Nodes:		    Using MoveIt to send pose commands

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import tf2_ros

from std_msgs.msg import Int16MultiArray
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

from linkattacher_msgs.srv import AttachLink, DetachLink

class pick_n_place(Node):

    def __init__(self, node):
        super().__init__('pick_n_place_subscriber')

        self.gripper_control = self.create_client(AttachLink, '/GripperMagnetON')
        self.marker_id_sub = self.create_subscription(Int16MultiArray, "/marker_ids", self.markeridcb, 10)

        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters for position and orientation
        self.declare_parameter("position", [0.35, 0.10, 0.68])
        self.declare_parameter("quat_xyzw", [0.5, 0.5, 0.5, 0.5])
        self.declare_parameter("cartesian", False)
        self.declare_parameter(
            "joint_positions",
            [
                0.0,
                -2.3911,
                2.4086,
                -3.1416,
                -1.5882,
                3.1416
            ],
        )

        # Create a MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=node,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        image_processing_rate = 0.02
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(image_processing_rate, self.pnpcb)

        self.marker_info_arr = None

    def markeridcb(self, data):
        marker_ids_arr = data.data
        arr = []
        for index, marker_id in enumerate(marker_ids_arr):
            try:
                transform = self.tf_buffer.lookup_transform('base_link', f'obj_{marker_id}', rclpy.time.Time())
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w
                position = [x, y, z]
                quat_xyzw = [qx, qy, qz, qw]
                arr.append([marker_id, position, quat_xyzw])
            except Exception as e:
                self.get_logger().error(f"Failed to process transform for obj_{marker_id}: {e}")
        self.marker_info_arr = arr

    def pnpcb(self):
        if self.marker_info_arr != None:
            print('focking')
            for index, marker_info in enumerate(self.marker_info_arr):
                try:
                    position = marker_info[1]
                    quat_xyzw = marker_info[2]

                    # Move to package pose
                    self.get_logger().info(
                        f"Moving to {{position: {position}, quat_xyzw: {quat_xyzw}}}"
                    )
                    self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=True)
                    self.moveit2.wait_until_executed()

                    # Attach link
                    while not self.gripper_control.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('EEF service not available, waiting again...')

                    req = AttachLink.Request()
                    req.model1_name =  f'obj_{marker_info[0]}'      
                    req.link1_name  = 'link'       
                    req.model2_name =  'ur5'       
                    req.link2_name  = 'wrist_3_link'  

                    self.gripper_control.call_async(req)

                    # Move to init pose
                    joint_positions = (
                        self.get_parameter_or("joint_positions").get_parameter_value().double_array_value
                    )
                    # Move to joint configuration
                    self.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
                    self.moveit2.move_to_configuration(joint_positions)
                    self.moveit2.wait_until_executed()

                    # Move to drop pose

                    # Detach link

                    # Move to init pose


                except Exception as e:
                    print('f')
                    self.get_logger().error(f"Failed to process marker ID {marker_info[0]}: {e}")


def main():
    rclpy.init()

    # Create a node for this example
    node = Node("path_execution")
    node.get_logger().info('Node created: path execution process')        # logging information

    pick_n_place_class = pick_n_place(node)
    rclpy.spin(pick_n_place_class)                                      # spining on the object to make it alive in ROS 2 DDS

    pick_n_place_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()
