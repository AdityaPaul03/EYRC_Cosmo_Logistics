#!/usr/bin/env python3

import sys
from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import numpy as np
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from std_msgs.msg import Int16MultiArray
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

class pick_n_place(Node):

    def __init__(self):

        super().__init__('pick_n_place_publisher')                                          # registering node
        callback_group = ReentrantCallbackGroup()

        ############ Topic SUBSCRIPTIONS ############
        self.marker_id_sub = self.create_subscription(Int16MultiArray, "/marker_ids", self.marker_planner, 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

        # Initialize message based on passed arguments
        self.twist_msg = TwistStamped()
        self.twist_msg.header.frame_id = ur5.base_link_name()
        self.twist_msg.twist.linear.x = 1.0
        self.twist_msg.twist.linear.y = 1.0
        self.twist_msg.twist.linear.z = 1.0
        self.twist_msg.twist.angular.x = 1.0
        self.twist_msg.twist.angular.y = 1.0
        self.twist_msg.twist.angular.z = 1.0

        ############ Constructor VARIABLES/OBJECTS ############

        planner_processing_rate = 0.02                                                     # rate of time to process image (seconds)
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(planner_processing_rate, self.process_planner)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.marker_ids_info = None



    def marker_planner(self, data):
        try:
            if self.marker_ids_info == None:
                info_arr = []
                for i in range(0,np.size(data.data)):
                    try:
                        obj_transform = self.tf_buffer.lookup_transform('base_link', f'obj_{data.data[i]}', rclpy.time.Time())
                        info_arr.append([data.data[i], obj_transform])

                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        self.get_logger().error("Failed to lookup transform.")
                self.marker_ids_info = info_arr
            else:
                pass

        except Exception as e:
            self.get_logger().error("Error getting current marker id: %s" % str(e))

    def process_planner(self):
        task_complete = False
        if self.marker_ids_info != None:
            for j in range(0, np.size(self.marker_ids_info)):
                while task_complete == False:
                    try:
                        obj_transform = self.marker_ids_info[j][1]
                        cam_transform = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
                        # print('done')
                        x = obj_transform.transform.translation.x - cam_transform.transform.translation.x
                        y = obj_transform.transform.translation.y - cam_transform.transform.translation.y
                        z = obj_transform.transform.translation.z - cam_transform.transform.translation.z
                        # print([x,y,z])

                        mag = pow(pow(x,2) + pow(y,2) + pow(z,2), 0.5)
                        diff_linear = [x/mag, y/mag, z/mag]
                        diff_angular = [0.0, 0.0, 0.0]

                        if mag > 0.01:
                            current_twist_msg = deepcopy(self.twist_msg)
                            current_twist_msg.header.stamp = self.get_clock().now().to_msg()
                            current_twist_msg.twist.linear.x *= diff_linear[0]
                            current_twist_msg.twist.linear.y *= diff_linear[1]
                            current_twist_msg.twist.linear.z *= diff_linear[2]
                            current_twist_msg.twist.angular.x *= diff_angular[0]
                            current_twist_msg.twist.angular.y *= diff_angular[1]
                            current_twist_msg.twist.angular.z *= diff_angular[2]
                            self.twist_pub.publish(current_twist_msg)
                            print('going')
                        else:
                            task_complete == True
                            print('done')

                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        self.get_logger().error("Failed to lookup transform.")

                task_complete == False


def main():
    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('pick_n_place_process')                    # creating ROS node

    node.get_logger().info('Node created: Pick n Place process')        # logging information

    pick_n_place_class = pick_n_place()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(pick_n_place_class)                                      # spining on the object to make it alive in ROS 2 DDS

    pick_n_place_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    main()
