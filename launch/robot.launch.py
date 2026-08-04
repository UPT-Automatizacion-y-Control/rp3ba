from launch import LaunchDescription
from launch_ros.actions import Node

DELAY_TIME = 1/120
KP_TBL, KI_TBL, KD_TBL = 150, 0, 0 # Ganancias PID para rehabilitador
#KP_TBL, KI_TBL, KD_TBL = 2500, 300, 1  # Ganancias PID para Ball on Plate

def generate_launch_description():

    trajectory_node = Node(
        package='rp3ba', executable='trajectory_node', name='Ref', output='screen',
        parameters=[{'delay_time': DELAY_TIME}], 
        remappings=[ ('trayectoria', 'pose') , ('platform_pose','pm')] )

    inv_kinematics_node = Node(
        package='rp3ba', executable='inv_kinematics_node.py', name='Inv_k', output='screen',
        remappings=[ ('angulos', 'qd') ] )
        
    u2d2_node = Node( 
        package='rp3ba', executable='u2d2_node', name='U2D2', output='screen', 
        parameters=[{'delay_time': DELAY_TIME, 'KP_TBL': KP_TBL, 'KI_TBL': KI_TBL, 'KD_TBL': KD_TBL}], 
        remappings=[ ('joints_goal', 'qd'), ('joints_state', 'qm') ]  )

    platform_state_node = Node(
        package='rp3ba', executable='platform_state_node.py', name='P_state', output='screen',
        remappings=[  ('joints_state', 'qm'), ('platform_pose','pm') ] )

    return LaunchDescription( [ 
        trajectory_node, 
        inv_kinematics_node, 
        u2d2_node,
        platform_state_node] )
