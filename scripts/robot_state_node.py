#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from numpy import sin, cos, pi
from array import array
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from rp3ba_interfaces.msg import StaticStateRP3BA, BaseStateRP3BA, JointStateRP3BA

DXL_IDS = (11, 12, 13, 21, 22, 23, 31, 32, 33) # JR
H0, H1, H2, H3, H4 = 0.175, 0.0667, 0.0784, 0.0848, 0.0916
HC1, HC2, HC3 = 0.046, 0.028, 0.032
I1X, I1Y, I1Z, I1XY = 7e-5, 10e-5, 8e-5, -1e-5
I2Y, I2Z = 2e-5, 2e-5
I3Y, I3Z = 2e-5, 2e-5
M1, M2, M3 = 0.14, 0.031, 0.031
MP  = 0.185
GM = 9.81
GW = np.array([0.0, 0.0, GM])
IP = np.array([ [34, -4, 0], [-4, 26, 0], [0, 0, 35] ])*1e-5

KP = 150./128. # KP_TBL/128. segun la documentación del MX28

class RobotStateNode(Node):

    def __init__(self):
        super().__init__('robot_state_node')
        
        self.ref_sub = self.create_subscription(JointState, 'reference_state', self.reference_callback, 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_data', self.joint_callback, 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, 'user_wrench', 10)
        self.state_pub = self.create_publisher(StaticStateRP3BA, 'static_state', 10)
        #self.joint_pub = self.create_publisher(JointStateRP3BA, 'joint_state', 10)
        #self.base_pub = self.create_publisher(BaseStateRP3BA, 'base_state', 10)
        
        self.reference_msg, self.base_msg = None, None
        
        self.filter_gain = 0.95
        self.prev_time, self.prev_Jbm_inv = None, None
        self.prev_Q, self.prev_Q_dot, self.prev_Q_ddot= None, np.zeros(9), np.zeros(9)
        self.q3_init = np.zeros(3)
        
        self.q0 = []
        for k in range(3):
            self.q0.append(k*2.*pi/3. + pi)
        
        self.wrench_msg = WrenchStamped() 
        self.wrench_msg.header.frame_id = "MB_Link"
        
        self.state_msg = StaticStateRP3BA() 
        self.state_msg.joints_name = ["jr11","jr12","jr13","jr21","jr22","jr23","jr31","jr32","jr33"]
        self.state_msg.joints_position = [0.0]*9 
        
        #self.joint_msg = JointStateRP3BA() 
        #self.joint_msg.joints_name = ["jr11","jr12","jr13","jr21","jr22","jr23","jr31","jr32","jr33"]
        #self.joint_msg.joints_position = [0.0]*9 
        #self.joint_msg.joints_velocity = [0.0]*9 
        #self.joint_msg.joints_acceleration = [0.0]*9 
        
        #self.base_msg = BaseStateRP3BA()
        
        self.rc = []
        self.rc_skew = []
        self.R_q0 = []
        self.q0 = []
        self.qc = []
        for k in range(3):
            qc_ = k*2.*pi/3.
            self.qc.append(qc_)
            q0_ = qc_ + pi
            self.q0.append(q0_)
            self.rc.append(H4*np.array([cos(qc_), sin(qc_), 0.]))
            self.rc_skew.append(np.array([[ 0.,       0.,       sin(qc_)],
                                          [ 0.,       0.,      -cos(qc_)],
                                          [-sin(qc_), cos(qc_), 0.      ]]))
            self.R_q0.append(np.array([[cos(q0_), -sin(q0_), 0.], 
                                       [sin(q0_),  cos(q0_), 0.], 
                                       [0.,        0.,       1.]]))
        
        self.get_logger().info("El nodo robot state esta corriendo")
        
    def reference_callback(self, msg):
        self.reference_msg = msg    
        
    def joint_callback(self, msg):     
        if self.reference_msg is None: return
        current_time = self.get_clock().now()
            
        # Posición articular
        joint_map = dict(zip(msg.name, msg.position))        
        q1 = np.array([ joint_map[f"dxl_{i}"] for i in DXL_IDS[:3] ])
        q2 = np.array([ joint_map[f"dxl_{i}"] for i in DXL_IDS[3:6] ])
        q3 = self.estimacion_q3(q1, q2)
        Q = np.array([q1, q2, q3]).reshape(-1)    
                 
        # Pose de la plataforma        
        r_ef = self.cinematica_brazos(q1, q2, q3)
        R_bm, r_bm = self.calculo_pose_base_movil(r_ef)
        quat_bm = Rotation.from_matrix(R_bm).as_quat()  
        
        self.publicar_static_state(Q, r_bm, quat_bm, msg.header.stamp)
           
        # Requisitos para calcular derivadas
        if self.prev_Q is None:
            self.prev_time, self.prev_Q = current_time, Q
            return
        
        # Cálculo de las derivadas articulares
        dt = (current_time - self.prev_time).nanoseconds *1e-9
        Q_dot = self.filter_gain*self.prev_Q_dot + (1. - self.filter_gain)*(Q - self.prev_Q) / dt
        Q_ddot = self.filter_gain*self.prev_Q_ddot + (1. - self.filter_gain)*(Q_dot - self.prev_Q_dot) / dt
        
        #if self.joint_pub.get_subscription_count() > 0:
        #    self.publicar_joint_state(Q, Q_dot, Q_ddot, msg.header.stamp)
       
        # Guardar estados previos
        self.prev_time, self.prev_Q, self.prev_Q_dot = current_time, Q, Q_dot

        # Twist de la plataforma móvil     
        Jvb = self.calculo_jacobiano_lineal_brazos(q1,q2,q3)
        Jbm = self.calculo_jacobiano_base_movil(Jvb)
        Jbm_inv = np.linalg.pinv(Jbm)
        xi = Jbm_inv @ Q_dot
                  
        # Requisitos para calcular aceleración
        if self.prev_Jbm_inv is None:
            self.prev_time, self.prev_Q, self.prev_Q_dot, self.prev_Jbm_inv = current_time, Q, Q_dot, Jbm_inv
            return
            
        # Aceleración de la plataforma móvil        
        Jbm_inv_dot = (Jbm_inv - self.prev_Jbm_inv) / dt
        xi_dot = Jbm_inv @ Q_ddot + Jbm_inv_dot @ Q_dot
        ap = xi_dot[:3] # aceleración lineal
        alpha = xi_dot[3:] # aceleración angular
        
        # Guardar estado previo
        self.prev_Jbm_inv = Jbm_inv
        
        #if self.base_pub.get_subscription_count() > 0:
        #    self.publicar_base_state(r_bm, quat_bm, xi, xi_dot, msg.header.stamp)
                                 
        # Pares de actuación (control P)
        reference_map = dict(zip(self.reference_msg.name, self.reference_msg.position))        
        q1_ref = np.array([ reference_map[f"dxl_{i}"] for i in DXL_IDS[:3] ])
        q2_ref = np.array([ reference_map[f"dxl_{i}"] for i in DXL_IDS[3:6] ])
        tau1 = KP*(q1_ref - q1) 
        tau2 = KP*(q2_ref - q2) 
        tau = np.array([tau1, tau2, np.zeros(3)])   
                
        # Dinámica de la plataforma
        Hb = self.calculo_inercia_brazos(q1, q2, q3)
        Cb = self.calculo_coriolis_brazos(q1, q2, q3, Q_dot[0:3], Q_dot[3:6], Q_dot[6:])
        Gb = self.calculo_gravedad_brazos(q1, q2, q3)
                
        Q_ddot = Q_ddot.reshape(3,3)
        for k in range(3):
            tau_dyn =  Hb[k] @ Q_ddot[:, k] + Cb[k]  + Gb[k]
                
        # Fuerzas de reacción en el sistema de coordenadas del mundo
        fr = []
        for k in range(3): 
            fr.append(self.R_q0[k] @ np.linalg.pinv(Jvb[k].T) @ (tau[:,k] - tau_dyn))
                
        # Fuerzas y pares del usuario
        fu = MP*(ap - GW) - np.sum(fr, axis=0) - np.array([0., 0., -1.55]) # offset por la fricción
        nu = ( IP @ alpha - np.sum( [np.cross(R_bm @ self.rc[k], fr[k]) for k in range(3)], axis=0) )
        
        self.publicar_wrench_state(fu, nu, msg.header.stamp)
                        
    # Funciones auxiliares
        
    def estimacion_q3(self, q1, q2):      
        def restriction_cost(q3):
            P = self.cinematica_brazos(q1, q2, q3)
            d = 2.0*H4*sin(pi/3.0)
            cost = np.sqrt( (np.linalg.norm(P[0] - P[1]) - d)**2 +
                               (np.linalg.norm(P[1] - P[2]) - d)**2 +
                               (np.linalg.norm(P[2] - P[0]) - d)**2 ) * 1000.0
            return cost
        result = minimize( restriction_cost, self.q3_init, method='Nelder-Mead', tol=1e-3,
            options={ 'maxiter': 1000, 'xatol': 1e-6, 'fatol': 1e-3, 'disp': False } )
        if not result.success:
            self.get_logger().warn( f"Configuration is not feasible: {result.message}" )       
        np.copyto(self.q3_init, result.x)
        return result.x
    
    def cinematica_brazos(self, q1_, q2_, q3_):      
        P = []
        for k in range(3):
            q0, q1, q2, q3 = self.q0[k], q1_[k], q2_[k], q3_[k]
            P.append( np.array([
                -H0*cos(q0) + (H2*cos(q2) - H3*sin(q2 + q3))*cos(q0 + q1),
                -H0*sin(q0) + (H2*cos(q2) - H3*sin(q2 + q3))*sin(q0 + q1),
                 H1 + H3*cos(q2 + q3) + H2*sin(q2)     ]) )           
        return P
        
    def calculo_pose_base_movil(self, r_ef):

        H = np.zeros((3, 3))
        r_bm = np.mean(r_ef, axis=0)

        for k in range(3):
            H += np.outer(self.rc[k], r_ef[k] - r_bm)

        U, _, Vt = np.linalg.svd(H)
        V = Vt.T

        I_ = np.eye(3)
        I_[2, 2] = np.linalg.det(V @ U.T)

        R_bm = V @ I_ @ U.T

        return R_bm, r_bm
        
    def calculo_jacobiano_lineal_brazos(self,q1_,q2_,q3_):
        Jvb = []
        for k in range(3):
            q1, q2, q3 = q1_[k], q2_[k], q3_[k]
            Jvb.append( np.array([
                [  sin(q1)*(HC3*sin(q2+q3)-H2*cos(q2)), -cos(q1)*(HC3*cos(q2+q3)+H2*sin(q2)), -HC3*cos(q2+q3)*cos(q1) ],
                [ -cos(q1)*(HC3*sin(q2+q3)-H2*cos(q2)), -sin(q1)*(HC3*cos(q2+q3)+H2*sin(q2)), -HC3*cos(q2+q3)*sin(q1) ],
                [  0.0, H2*cos(q2)-HC3*sin(q2+q3), -HC3*sin(q2+q3) ]  ]))
        return Jvb
        
    def calculo_jacobiano_base_movil(self, Jvb):   
        Jbm = np.zeros((9,6))
        for k in range(3):
            Jbm[0+3*k:3+3*k,0:3] = np.linalg.inv(self.R_q0[k] @ Jvb[k])
            Jbm[0+3*k:3+3*k,3:6] = self.rc_skew[k].T     
        return Jbm

    def calculo_inercia_brazos(self,q1_,q2_,q3_):
        H = []
        for k in range(3):
            q2, q3 = q2_[k], q3_[k]
            H11 = ( I1Y + 0.5*I2Y + 0.5*I3Y + 0.5*I2Y*cos(2*q2) - 0.5*I3Y*cos(2*q2 + 2*q3) + 0.5*H2**2*M3 + 0.5*HC2**2*M2
                  + 0.5*HC3**2*M3 + 0.5*H2**2*M3*cos(2*q2) + 0.5*HC2**2*M2*cos(2*q2) - 0.5*HC3**2*M3*cos(2*q2 + 2*q3) 
                  - H2*HC3*M3*sin(q3) - H2*HC3*M3*sin(2*q2 + q3) )
            H22 = M3*H2**2 - 2*M3*H2*HC3*sin(q3) + M2*HC2**2 + M3*HC3**2 + I2Z + I3Z 
            H23 = M3*HC3**2 - H2*M3*HC3*sin(q3) + I3Z 
            H33 = M3*HC3**2 + I3Z
            
            H.append(np.array([ [H11, 0.0, 0.0], [0.0, H22, H23], [0.0, H23, H33] ] ))
        return H  
        
    def calculo_coriolis_brazos(self,q1_,q2_,q3_,q1_dot,q2_dot,q3_dot):
        C = []
        for k in range(3):
            q1, q2, q3 = q1_[k], q2_[k], q3_[k]
            q1d, q2d, q3d = q1_dot[k], q2_dot[k], q3_dot[k]
            C.append(np.array([
                -q1d * ( - I3Y*q2d*sin(2*q2 + 2*q3) - I3Y*q3d*sin(2*q2 + 2*q3) + I2Y*q2d*sin(2*q2) + H2**2*M3*q2d*sin(2*q2)
                 + HC2**2*M2*q2d*sin(2*q2) - HC3**2*M3*q2d*sin(2*q2 + 2*q3) - HC3**2*M3*q3d*sin(2*q2 + 2*q3)
                 + 2*H2*HC3*M3*q2d*cos(2*q2 + q3) + H2*HC3*M3*q3d*cos(2*q2 + q3) + H2*HC3*M3*q3d*cos(q3)  ), 
                (  - 0.5*I3Y*q1d**2*sin(2*q2 + 2*q3) + 0.5*I2Y*q1d**2*sin(2*q2) - 0.5*HC3**2*M3*q1d**2*sin(2*q2 + 2*q3) 
                + 0.5*H2**2*M3*q1d**2*sin(2*q2) + 0.5*HC2**2*M2*q1d**2*sin(2*q2) - H2*HC3*M3*q3d**2*cos(q3) 
                + H2*HC3*M3*q1d**2*cos(2*q2 + q3) - 2*H2*HC3*M3*q2d*q3d*cos(q3) ), 
                (  - 0.5*I3Y*q1d**2*sin(2*q2 + 2*q3) - 0.5*HC3**2*M3*q1d**2*sin(2*q2 + 2*q3)
                 + 0.5*H2*HC3*M3*q1d**2*cos(q3) + H2*HC3*M3*q2d**2*cos(q3) + 0.5*H2*HC3*M3*q1d**2*cos(2*q2 + q3) ) ]) )
        return C
        
    def calculo_gravedad_brazos(self, q1_, q2_, q3_):
        G = []
        for k in range(3):
            q1, q2, q3 = q1_[k], q2_[k], q3_[k]
            G.append(np.array([ 0, H2*M3*cos(q2)+HC2*M2*cos(q2)-HC3*M3*sin(q2+q3), -HC3*M3*sin(q2+q3)])*GM )
        return G
        
    def publicar_static_state(self, Q, r_bm, quat_bm, stamp):     
        # Asignar datos articulares al mensaje static_state
        self.state_msg.header.stamp = stamp
        self.state_msg.joints_position[:] = array('d', Q)
              
        # Desempaquetar los elementos de la pose
        ppp = self.state_msg.platform_pose.position
        ppo = self.state_msg.platform_pose.orientation
                
        # Asignar datos de la pose al mensaje static_state
        ppp.x, ppp.y, ppp.z = map(float, r_bm)  
        ppo.x, ppo.y, ppo.z, ppo.w = map(float, quat_bm)
                
        #Publicar Parallel Static State
        self.state_pub.publish(self.state_msg)  
        
    def publicar_joint_state(self, Q, Q_dot, Q_ddot, stamp):
        # Asignar datos articulares al mensaje Joint State
        self.joint_msg.header.stamp = stamp
        self.joint_msg.joints_position[:] = array('d', Q)
        self.joint_msg.joints_velocity[:] = array('d', Q_dot)
        self.joint_msg.joints_acceleration[:] = array('d', Q_ddot)
                    
        # Publicar Joint State
        self.joint_pub.publish(self.joint_msg)  
        
    def publicar_base_state(self, r_bm, quat_bm, xi, xi_dot, stamp):
        # Desempaquetar los elementos del mensaje base_state
        ppp = self.base_msg.platform_pose.position
        ppo = self.base_msg.platform_pose.orientation
        ptl = self.base_msg.platform_twist.linear
        pta = self.base_msg.platform_twist.angular
        pal = self.base_msg.platform_accel.linear
        paa = self.base_msg.platform_accel.angular
            
        # Asignar datos articulares al mensaje Base State
        self.base_msg.header.stamp = stamp
        ppp.x, ppp.y, ppp.z = map(float, r_bm)  
        ppo.x, ppo.y, ppo.z, ppo.w = map(float, quat_bm)
        ptl.x, ptl.y, ptl.z = map(float, xi[:3])
        pta.x, pta.y, pta.z = map(float, xi[3:])
        pal.x, pal.y, pal.z = map(float, xi_dot[:3])
        paa.x, paa.y, paa.z = map(float, xi_dot[3:])  
                    
        # Publicar Base State
        self.base_pub.publish(self.base_msg)  
        
    def publicar_wrench_state(self, fu, nu, stamp):
        # Desempaquetar los elementos del mensaje user_wrench
        uwf = self.wrench_msg.wrench.force
        uwt = self.wrench_msg.wrench.torque
                
        # Asignar datos al mensaje user_wrench
        uwf.x, uwf.y, uwf.z = map(float, fu)
        uwt.x, uwt.y, uwt.z = map(float, nu)  
                
        # Publicar wrench_msg
        self.wrench_msg.header.stamp = stamp
        self.wrench_pub.publish(self.wrench_msg)   

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateNode()
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
