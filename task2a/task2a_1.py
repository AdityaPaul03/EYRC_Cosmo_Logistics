#!/usr/bin/env python3

'''---
transforms:
- header:
    stamp:
      sec: 1697830439
      nanosec: 876184713
    frame_id: camera_link
  child_frame_id: cam_3
  transform:
    translation:
      x: 1.251
      y: -0.4557662216411074
      z: -0.06314226209913648
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---
transforms:
- header:
    stamp:
      sec: 1697830439
      nanosec: 876506934
    frame_id: base_link
  child_frame_id: obj_3
  transform:
    translation:
      x: 0.22242465934327194
      y: -0.45576622164110736
      z: 0.6565041388153078
    rotation:
      x: 0.7070860047312805
      y: 0.005420508569826063
      z: 0.005420508569826064
      w: 0.7070860047312806
---
transforms:
- header:
    stamp:
      sec: 1697830439
      nanosec: 876666545
    frame_id: camera_link
  child_frame_id: cam_49
  transform:
    translation:
      x: 1.253
      y: 0.5701980807920453
      z: -0.06189760877034591
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---
transforms:
- header:
    stamp:
      sec: 1697830439
      nanosec: 876788724
    frame_id: base_link
  child_frame_id: obj_49
  transform:
    translation:
      x: 0.2245914345954736
      y: 0.5701980807920453
      z: 0.6568660568905239
    rotation:
      x: -0.003474440997636341
      y: 0.7070982451256359
      z: 0.707098245125636
      w: -0.0034744409976363412
---
transforms:
- header:
    stamp:
      sec: 1697830439
      nanosec: 876910575
    frame_id: camera_link
  child_frame_id: cam_1
  transform:
    translation:
      x: 1.396
      y: 0.05621881083881482
      z: -0.02323710848004346
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---
transforms:
- header:
    stamp:
      sec: 1697830439
      nanosec: 877018490
    frame_id: base_link
  child_frame_id: obj_1
  transform:
    translation:
      x: 0.3727245887544679
      y: 0.05621881083881482
      z: 0.6573165250959244
    rotation:
      x: 0.4999893849611907
      y: 0.5000106148134559
      z: 0.500010614813456
      w: 0.49998938496119083
---
'''


from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

# Initialize message based on passed arguments

__twist_msg = TwistStamped()
__twist_msg.header.frame_id = ur5.base_link_name()
__twist_msg.twist.linear.x = 1.0
__twist_msg.twist.linear.y = 1.0
__twist_msg.twist.linear.z = 1.0
__twist_msg.twist.angular.x = 1.0
__twist_msg.twist.angular.y = 1.0
__twist_msg.twist.angular.z = 1.0

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_servo")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    __twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    linear = [0.0, 0.0, -0.22]
    angular = [0.0, 0.0, 0.0]

    def servo_circular_motion():
        """Move in a circular motion using Servo"""
        twist_msg = deepcopy(__twist_msg)
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.twist.linear.x *= linear[0]
        twist_msg.twist.linear.y *= linear[1]
        twist_msg.twist.linear.z *= linear[2]
        twist_msg.twist.angular.x *= angular[0]
        twist_msg.twist.angular.y *= angular[1]
        twist_msg.twist.angular.z *= angular[2]
        __twist_pub.publish(twist_msg)

    # Create timer for moving in a circular motion
    node.create_timer(0.02, servo_circular_motion)

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
