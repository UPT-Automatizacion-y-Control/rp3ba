#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from rp3ba_interfaces.msg import StaticStateRP3BA

JR_IDS = (11, 12, 13, 21, 22, 23, 31, 32, 33) # JointRobot

class VisualizationDataNode(Node):
    def __init__(self):
        super().__init__('vis_data')

        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = 'World_Link'
        self.transform_stamped.child_frame_id = 'MB_Link'
        self.transform_broadcaster = TransformBroadcaster(self)
        
        self.sub = self.create_subscription( StaticStateRP3BA,'static_state', self.state_callback, 10 )
        self.pub = self.create_publisher(JointState,'joint_state_viz', 10)
        
        self.joints_viz_msg = JointState()
        self.joints_viz_msg.name  = [f"dxl_{dxl_id}" for dxl_id in JR_IDS]
        self.joints_viz_msg.position = [0.0]*9 
               
        self.get_logger().info("El nodo visualizacion de datos esta corriendo")

    def state_callback(self, msg): 
        self.joints_viz_msg.header = msg.header        
        self.joints_viz_msg.position[:] = msg.joints_position
        self.pub.publish( self.joints_viz_msg)
        
        self.transform_stamped.header.stamp = msg.header.stamp        
        self.transform_stamped.transform.translation.x = msg.platform_pose.position.x
        self.transform_stamped.transform.translation.y = msg.platform_pose.position.y
        self.transform_stamped.transform.translation.z = msg.platform_pose.position.z
        self.transform_stamped.transform.rotation = msg.platform_pose.orientation
        
        self.transform_broadcaster.sendTransform(self.transform_stamped)
        
def main(args=None):
    rclpy.init(args=args)
    node = VisualizationDataNode()
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
