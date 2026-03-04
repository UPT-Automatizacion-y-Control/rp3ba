#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy
from math import cos, sin, pi
from numpy import array
from numpy.linalg import norm
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

DXL_IDS = (10, 20, 30, 11, 21, 31, 12, 22, 32)
h = (0.175, 0.0667, 0.0784, 0.0848, 0.0916)

class InvKinematicsNode(Node):
    def __init__(self):
        super().__init__('inv_kinematics')
        
        self.pub = self.create_publisher(JointState, 'angulos', 1000)
        self.sub = self.create_subscription(Twist, 'pose', self.pose_callback, 10)
        
        self.msg_out = JointState()
        self.msg_out.header = Header()
        self.msg_out.header.stamp = self.get_clock().now().to_msg()
        self.msg_out.header.frame_id = "base"
        self.msg_out.name = [f"{dxl_id}_Joint" for dxl_id in DXL_IDS]
            
        self.msg_out.position = [0.0] * 9 
        
        self.get_logger().info("El nodo Inverse kinematics esta corriendo")

    def pose_callback(self, data):
        
        P_c_m = array([[h[4], -h[4]*cos(pi/3), -h[4]*cos(pi/3)], 
                                 [    0,   h[4]*sin(pi/3), -h[4]*sin(pi/3)],
                                 [    0,                      0,                      0]])

        P_O_f = array([[h[0], -h[0]*cos(pi/3), -h[0]*cos(pi/3)],
                                [    0,   h[0]*sin(pi/3), -h[0]*sin(pi/3)],
                                [    0,                      0,                      0]])
        
        Px, Py, Pz = data.linear.x,  data.linear.y,  data.linear.z 
        Tx, Ty, Tz = data.angular.x, data.angular.y, data.angular.z
        
        T_m_f = array([[cos(Ty)*cos(Tz), cos(Tz)*sin(Tx)*sin(Ty)-cos(Tx)*sin(Tz), sin(Tx)*sin(Tz)+cos(Tx)*cos(Tz)*sin(Ty),  Px],
                       [cos(Ty)*sin(Tz), cos(Tx)*cos(Tz)+sin(Tx)*sin(Ty)*sin(Tz), cos(Tx)*sin(Ty)*sin(Tz)-cos(Tz)*sin(Tx),  Py],
                       [-sin(Ty),        cos(Ty)*sin(Tx),                         cos(Tx)*cos(Ty),                          Pz]])  
                       
        
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
