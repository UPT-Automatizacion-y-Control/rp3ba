#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState

DXL_IDS = (11, 12, 13, 21, 22, 23, 31, 32, 33) # JR

class U2D2VirtualNode(Node):
    def __init__(self):
        super().__init__('pose_to_tf')

        self.joints_sub = self.create_subscription( JointState,'joints_goal', self.joints_callback, 10 )
        self.joints_pub = self.create_publisher( JointState, 'joints_state', 10 )

        self.get_logger().info("El nodo u2d2 virtual esta corriendo")

    def joints_callback(self, msg):
        # TODO: Programar dinámica
        self.joints_pub.publish(msg)

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
