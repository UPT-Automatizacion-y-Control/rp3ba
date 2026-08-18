#!/usr/bin/env python3

import rclpy
import time
import numpy as np
from numpy import sin, cos
from array import array
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

DXL_IDS = (11, 12, 13, 21, 22, 23, 31, 32, 33) # JR
H0, H1, H2, H3, H4 = 0.175, 0.0667, 0.0784, 0.0848, 0.0916

class DataVizNode(Node):
    def __init__(self):
        super().__init__('pose_to_tf')

        self.declare_parameter('parent_frame', 'World_Link')
        self.declare_parameter('child_frame', 'MB_Link')
        self.parent_frame = self.get_parameter( 'parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter( 'child_frame').get_parameter_value().string_value
        self.br = TransformBroadcaster(self)
        self.joints_sub = self.create_subscription( JointState,'joints_state', self.joints_callback, 10 )
        self.joints_pub = self.create_publisher( JointState, 'joints_state_full', 10 )
        
        self.Q30 = np.zeros(3)

        self.get_logger().info("El nodo data for vizualization esta corriendo")

    def joints_callback(self, msg):
        joint_map = dict(zip(msg.name, msg.position))        
        Q1 = np.array([ joint_map[f"dxl_{i}"] for i in DXL_IDS[0:3] ])
        Q2 = np.array([ joint_map[f"dxl_{i}"] for i in DXL_IDS[3:6] ])
        Q3 = self.estimacion_Q3(Q1, Q2)
        np.copyto(self.Q30,Q3)
        
        joints_full_msg = JointState()
        joints_full_msg.header = msg.header
        joints_full_msg.name  = [f"dxl_{dxl_id}" for dxl_id in DXL_IDS]
        joints_full_msg.position = array('d', np.concatenate((Q1, Q2, Q3+1.5708))) #Se agregaron pi/2 para el display del tercer eslabon
        self.joints_pub.publish(joints_full_msg)
        
        tr = TransformStamped()
        tr.header.stamp = msg.header.stamp
        tr.header.frame_id = self.parent_frame
        tr.child_frame_id = self.child_frame
        
        Pef = self.cinematica_brazos(Q1, Q2, Q3)
        R_mat, t_vec = self.calculo_pose(Pef)
        quat = Rotation.from_matrix(R_mat).as_quat()  
        
        ppp = tr.transform.translation
        ppo = tr.transform.rotation
        ppp.x, ppp.y, ppp.z = map(float, t_vec)  
        ppo.x, ppo.y, ppo.z, ppo.w = map(float, quat)
        
        self.br.sendTransform(tr)
        
    def estimacion_Q3(self, Q1, Q2):
        
        def restriction_cost(Q3):
            P = self.cinematica_brazos(Q1, Q2, Q3)
            d = 2 * H4 * sin(2 * np.pi / 6)
            cost = np.sqrt( (np.linalg.norm(P[:, 0] - P[:, 1]) - d)**2 +
                               (np.linalg.norm(P[:, 1] - P[:, 2]) - d)**2 +
                               (np.linalg.norm(P[:, 2] - P[:, 0]) - d)**2 ) * 1000.0
            return cost
        
        result = minimize( restriction_cost, self.Q30, method='Nelder-Mead', tol=1e-3,
            options={ 'maxiter': 1000, 'xatol': 1e-6, 'fatol': 1e-3, 'disp': False } )
        
        if not result.success:
            self.get_logger().warn( f"Configuration is not feasible: {result.message}" )
        
        return result.x
        
    def cinematica_brazos(self, Q1, Q2, Q3):
        
        P = np.zeros((3, 3))

        for k in range(3):
            Q0k = np.pi + k * 2 * np.pi / 3
            P[:, k] = np.array([
                -H0 * cos(Q0k) + (H2 * cos(Q2[k]) - H3 * sin(Q2[k] + Q3[k])) * cos(Q0k + Q1[k]),
                -H0 * sin(Q0k) + (H2 * cos(Q2[k]) - H3 * sin(Q2[k] + Q3[k])) * sin(Q0k + Q1[k]),
                H1 + H3 * cos(Q2[k] + Q3[k]) + H2 * sin(Q2[k])     ])
                    
        return P

    def calculo_pose(self, P_b):

        H = np.zeros((3, 3))
        C_b = np.mean(P_b, axis=1)

        P_i = np.array([ [H4, H4*cos(2*np.pi/3), H4*cos(-2*np.pi/3)],
                         [0,  H4*sin(2*np.pi/3), H4*sin(-2*np.pi/3)],
                         [0,  0,                 0                 ] ])

        C_i = np.mean(P_i, axis=1)

        for k in range(3):
            H += np.outer(P_i[:, k] - C_i, P_b[:, k] - C_b)

        U, _, Vt = np.linalg.svd(H)
        V = Vt.T

        I_ = np.eye(3)
        I_[2, 2] = np.linalg.det(V @ U.T)

        R_mat = V @ I_ @ U.T
        t_vec = C_b - R_mat @ C_i

        return R_mat, t_vec

def main(args=None):
    rclpy.init(args=args)
    node = DataVizNode()
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
