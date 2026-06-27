#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

DXL_IDS = (11, 12, 13, 21, 22, 23, 31, 32, 33) # JR

class U2D2VirtualNode(Node):
    def __init__(self):
        super().__init__('pose_to_tf')

        self.declare_parameter('parent_frame', 'World_Link')
        self.declare_parameter('child_frame', 'MB_Link')
        self.parent_frame = self.get_parameter( 'parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter( 'child_frame').get_parameter_value().string_value
        self.br = TransformBroadcaster(self)
        self.pose_sub = self.create_subscription( PoseStamped, 'pose',  self.pose_callback, 10 )
        self.joints_sub = self.create_subscription( JointState,'joints_goal', self.joints_callback, 10 )
        self.joints_pub = self.create_publisher( JointState, 'joints_state', 10 )
        
        time.sleep(2.0)
        q0 = JointState()
        q0.header = Header()
        q0.header.stamp = self.get_clock().now().to_msg()
        q0.header.frame_id = "World_Link"
        q0.name = [f"dxl_{dxl_id}" for dxl_id in DXL_IDS]            
        q0.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.57, 1.57, 1.57]  
        self.joints_pub.publish(q0)

        self.get_logger().info("El nodo u2d2 virtual esta corriendo")

    def joints_callback(self, msg):
        self.joints_pub.publish(msg)

    def pose_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = U2D2VirtualNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
