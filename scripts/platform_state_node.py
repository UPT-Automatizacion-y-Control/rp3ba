#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

DXL_IDS = (11, 12, 13, 21, 22, 23) # JR
h = (0.175, 0.0667, 0.0784, 0.0848, 0.0916)

class PlatformStateNode(Node):

    def __init__(self):
        super().__init__('platform_state_node')
        
        self.subscription = self.create_subscription(  JointState, 'joints_state', self.joint_callback, 10)
        self.pose_pub  = self.create_publisher(PoseStamped,  'platform_pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, 'platform_twist', 10)
        self.accel_pub = self.create_publisher(AccelStamped, 'platform_accel', 10)
        self.joint_kin_pub = self.create_publisher( JointTrajectoryPoint, '/joint_kinematics', 10 )
        
        self.prev_time, self.prev_dq , self.prev_J = None, None, None
        self.get_logger().info("El nodo platform state esta corriendo")

        
    def joint_callback(self, msg):    
        current_time = msg.header.stamp
        
        # Entradas de posición
        joint_map = dict(zip(msg.name, msg.position))        
        Q1 = np.array([ joint_map[f"dxl_{i}"] for i in DXL_IDS[0:3] ])
        Q2 = np.array([ joint_map[f"dxl_{i}"] for i in DXL_IDS[3:6] ])

        # Calculo de pose
        Q3 = self.estimacion_Q3(Q1, Q2)
        Pef = self.cinematica_brazos(Q1, Q2, Q3)
        R_mat, t = self.calculo_pose(Pef)
        pose_msg = self.crear_msg_pose(R_mat, t, current_time)
        self.pose_pub .publish(pose_msg)
        
        # Entradas de velocidad
        if not msg.velocity: return
        vel_map   = dict(zip(msg.name, msg.velocity))
        dQ1 = np.array([vel_map[f"dxl_{i}"] for i in DXL_IDS[0:3]])
        dQ2 = np.array([vel_map[f"dxl_{i}"] for i in DXL_IDS[3:6]])

        # Cálculo de twist     
        J = self.calcular_jacobiano(Q1, Q2, Q3)
        dq = np.concatenate([dQ1, dQ2])   
        xi = J @ dq
        twist_msg = self.crear_msg_twist(xi, current_time)
        self.twist_pub.publish(twist_msg)
        
        # Requisitos para calcular accel
        t_sec = self.get_clock().now().to_msg().sec
        if self.prev_time is None:
            self.prev_time, self.prev_dq, self.prev_J = t_sec, dq, J
            return
        dt = t_sec - self.prev_time
        if dt <= 0.0:
            return

        # Cálculo de accel        
        ddq = (dq - self.prev_dq) / dt
        Jdot = (J - self.prev_J) / dt
        xi_dot = J @ ddq + Jdot @ dq
        accel_msg = self.crear_msg_accel(xi_dot, current_time)
        self.accel_pub.publish(accel_msg)

        # Guardar estados previos
        self.prev_time, self.prev_dq, self.prev_J = t_sec, dq, J
        
    def crear_msg_pose(self, R_mat, t, current_time):
        quat = Rotation.from_matrix(R_mat).as_quat()  
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "World_Link"
        
        pose_msg.pose.position.x = float(t[0])
        pose_msg.pose.position.y = float(t[1])
        pose_msg.pose.position.z = float(t[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        
        return pose_msg
        
    def  crear_msg_twist(self, xi, current_time):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = current_time
        twist_msg.header.frame_id = "World_Link"

        twist_msg.twist.linear.x  = float(xi[0])
        twist_msg.twist.linear.y  = float(xi[1])
        twist_msg.twist.linear.z  = float(xi[2])
        twist_msg.twist.angular.x = float(xi[3])
        twist_msg.twist.angular.y = float(xi[4])
        twist_msg.twist.angular.z = float(xi[5])

        return twist_msg
        
    def  crear_msg_accel(self, xi_dot, current_time):
            accel_msg = AccelStamped()
            accel_msg.header.stamp = current_time
            accel_msg.header.frame_id = "World_Link"

            accel_msg.accel.linear.x  = float(xi_dot[0])
            accel_msg.accel.linear.y  = float(xi_dot[1])
            accel_msg.accel.linear.z  = float(xi_dot[2])
            accel_msg.accel.angular.x = float(xi_dot[3])
            accel_msg.accel.angular.y = float(xi_dot[4])
            accel_msg.accel.angular.z = float(xi_dot[5])
            
            return accel_msg

    def estimacion_Q3(self, Q1, Q2):
        
        def restriction_cost(Q3):
            P = self.cinematica_brazos(Q1, Q2, Q3)
            d = 2 * h[4] * np.sin(2 * np.pi / 6)
            J = np.sqrt( (np.linalg.norm(P[:, 0] - P[:, 1]) - d)**2 +
                               (np.linalg.norm(P[:, 1] - P[:, 2]) - d)**2 +
                               (np.linalg.norm(P[:, 2] - P[:, 0]) - d)**2 ) * 1000.0
            return J
        
        result = minimize( restriction_cost, np.zeros(3), method='Nelder-Mead', tol=1e-3,
            options={ 'maxiter': 1000, 'xatol': 1e-6, 'fatol': 1e-3, 'disp': False } )
        
        if not result.success:
            self.get_logger().warn( f"Configuration is not feasible: {result.message}" )
        
        return result.x
    
    def cinematica_brazos(self, Q1, Q2, Q3):
        
        P = np.zeros((3, 3))

        for k in range(3):
            Q0k = np.pi + k * 2 * np.pi / 3
            P[:, k] = np.array([
                -h[0] * np.cos(Q0k) + (h[2] * np.cos(Q2[k]) - h[3] * np.sin(Q2[k] + Q3[k])) * np.cos(Q0k + Q1[k]),
                -h[0] * np.sin(Q0k) + (h[2] * np.cos(Q2[k]) - h[3] * np.sin(Q2[k] + Q3[k])) * np.sin(Q0k + Q1[k]),
                h[1] + h[3] * np.cos(Q2[k] + Q3[k]) + h[2] * np.sin(Q2[k])     ])
                    
        return P

    def calculo_pose(self, P_b):

        H = np.zeros((3, 3))
        C_b = np.mean(P_b, axis=1)

        P_i = np.array([ [h[4], h[4]*np.cos(2*np.pi/3), h[4]*np.cos(-2*np.pi/3)],
                                  [0,     h[4]*np.sin(2*np.pi/3), h[4]*np.sin(-2*np.pi/3)],
                                  [0,     0,                                   0] ])

        C_i = np.mean(P_i, axis=1)

        for k in range(3):
            H += np.outer(P_i[:, k] - C_i, P_b[:, k] - C_b)

        U, _, Vt = np.linalg.svd(H)
        V = Vt.T

        I_ = np.eye(3)
        I_[2, 2] = np.linalg.det(V @ U.T)

        R_mat = V @ I_ @ U.T
        t = C_b - R_mat @ C_i

        return R_mat, t
        
    def calcular_jacobiano(self, Q1, Q2, Q3, eps=1e-6):

        q = np.concatenate([Q1, Q2])
        J = np.zeros((6, 6))

        R0, t0 = self.calculo_pose(self.cinematica_brazos(Q1, Q2, Q3))
        x0 = self.pose_to_vector(R0, t0)

        for i in range(6):

            q_pert = q.copy()
            q_pert[i] += eps

            Q1p = q_pert[0:3]
            Q2p = q_pert[3:6]

            Q3p = self.estimacion_Q3(Q1p, Q2p)
            Rp, tp = self.calculo_pose(self.cinematica_brazos(Q1p, Q2p, Q3p))
            xp = self.pose_to_vector(Rp, tp)

            J[:, i] = (xp - x0) / eps

        return J

    def pose_to_vector(self, R, t):

        rotvec = Rotation.from_matrix(R).as_rotvec()
        return np.hstack((t, rotvec))

def main(args=None):
    rclpy.init(args=args)
    node = PlatformStateNode()
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
