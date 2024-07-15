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
# Nodes:		    Using MoveIt to sent pose commands

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5


def main():
    rclpy.init()

    # Create node for this example
    node = Node("path_execution")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.35, 0.10, 0.68])
    node.declare_parameter("quat_xyzw", [0.5, 0.5, 0.5, 0.5])
    node.declare_parameter("cartesian", False)
    node.declare_parameter(
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

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Define the pose sequence
    pose_sequence = [
        ([0.35, 0.10, 0.68], [0.5, 0.5, 0.5, 0.5]),
        ([-0.37, 0.12, 0.397], [0.0, -0.7071067811865475, 0.0, 0.7071067811865476]),
        ([0.194, -0.43, 0.701], [0.7071067811865475, 0.0, 0.0, 0.7071067811865475]),
        ([-0.37, 0.12, 0.397], [0.0, -0.7071067811865475, 0.0, 0.7071067811865476])
    ]

    for position, quat_xyzw in pose_sequence:

        node.get_logger().info(
            f"Moving to {{position: {position}, quat_xyzw: {quat_xyzw}}}"
        )
        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=False)
        moveit2.wait_until_executed()
        
        joint_positions = (
            node.get_parameter_or("joint_positions").get_parameter_value().double_array_value
        )
        # Move to joint configuration
        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
        moveit2.move_to_configuration(joint_positions)
        moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()

