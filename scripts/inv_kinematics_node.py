#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy
from math import cos, sin, pi
from numpy import array
from numpy.linalg import norm
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion

DXL_IDS = (11, 12, 13, 21, 22, 23) # JR
h = (0.175, 0.0667, 0.0784, 0.0848, 0.0916)

class InvKinematicsNode(Node):
    def __init__(self):
        super().__init__('inv_kinematics_node')
        
        self.pub = self.create_publisher(JointState, 'angulos', 1000)
        self.sub = self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        
        self.msg_out = JointState()
        self.msg_out.header = Header()
        self.msg_out.header.stamp = self.get_clock().now().to_msg()
        self.msg_out.header.frame_id = "World_Link"
        self.msg_out.name = [f"{dxl_id}_Joint" for dxl_id in DXL_IDS]
            
        self.msg_out.position = [0.0] * 9 
        
        self.get_logger().info("El nodo inverse kinematics esta corriendo")

    def pose_callback(self, msg):
        
        p = msg.pose.position
        q = msg.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        T_m_f = array([[cos(pitch)*cos(yaw), cos(yaw)*sin(roll)*sin(pitch)-cos(roll)*sin(yaw),  sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch),  p.x],
                                [cos(pitch)*sin(yaw),  cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw), cos(roll)*sin(pitch)*sin(yaw)-cos(yaw)*sin(roll),   p.y],
                                [-sin(pitch),                 cos(pitch)*sin(roll),                                               cos(roll)*cos(pitch),                                                p.z]])  
                           
        P_c_m = array([[h[4], -h[4]*cos(pi/3), -h[4]*cos(pi/3)], 
                                 [    0,   h[4]*sin(pi/3), -h[4]*sin(pi/3)],
                                 [    0,                      0,                      0]])

        P_O_f = array([[h[0], -h[0]*cos(pi/3), -h[0]*cos(pi/3)],
                                [    0,   h[0]*sin(pi/3), -h[0]*sin(pi/3)],
                                [    0,                      0,                      0]])
        q0_offset = array([-pi, pi/3, -pi/3])
        
        for i in range(3):
            try:
                P_q4_q2 = T_m_f[:,0:3] @ P_c_m[:,i] + T_m_f[:,3] - P_O_f[:,i] - array([0, 0, h[1]])
                d_q4_q2 = norm(P_q4_q2) 
                theta_q4_q2 = math.asin(P_q4_q2[2]/d_q4_q2)
                alpha = -math.acos((pow(h[3],2) - pow(h[2],2) - pow(d_q4_q2,2))/(-2*h[2]*d_q4_q2))
            except ValueError:
                self.get_logger().info("No se pudo calcular la cinemática inversa")
                return
                
            self.msg_out.position[i+0] = math.atan2(P_q4_q2[1], P_q4_q2[0]) + q0_offset[i] 
            self.msg_out.position[i+3] = alpha + theta_q4_q2
            self.msg_out.position[i+6] = -math.acos((pow(d_q4_q2,2) - pow(h[2],2) - pow(h[3],2))/(-2*h[2]*h[3])) - pi
            
        for i in range(9):
            if self.msg_out.position[i] < -pi:
                self.msg_out.position[i] += 2*pi
            elif self.msg_out.position[i] > pi:
                self.msg_out.position[i] -= 2*pi

        self.msg_out.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = InvKinematicsNode()
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
