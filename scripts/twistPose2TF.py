#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class TwistPoseToTF(Node):

    def __init__(self):
        super().__init__('twist_pose_to_tf')
        self.declare_parameter('parent_frame', 'parent_Link')
        self.declare_parameter('child_frame', 'child_Link')
       
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.subscription = self.create_subscription( Twist, '/pose_twist', self.twist_callback, 10 )
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info( f'Publicando TF: {self.parent_frame} → {self.child_frame}' )

    def twist_callback(self, msg: Twist):
    
        x, y, z = msg.linear.x, msg.linear.y, msg.linear.z
        roll, pitch, yaw  = msg.angular.x, msg.angular.y, msg.angular.z

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = TwistPoseToTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

