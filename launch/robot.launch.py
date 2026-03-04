from launch import LaunchDescription
from launch_ros.actions import Node

DELAY_TIME = 1/70

def generate_launch_description():

    trajectory_node = Node(
        package='rp3ba', executable='trayectoria_r', name='ref', output='screen',
        parameters=[{'delay_time': DELAY_TIME}], remappings=[ ('trayectoria', 'pose') ] )

    twist_mean_node = Node(
        package='rp3ba', executable='twist_mean_filter_node', name='ref_mean', output='screen',
        remappings=[ ('twist_in', 'pose'), ('twist_out', 'pose_fil') ] )

    inv_k_node = Node(
        package='rp3ba', executable='inv_kinematics.py', name='Inv_k', output='screen',
        remappings=[ ('pose', 'pose_fil'), ('angulos', 'qd') ] )
        
    u2d2_node = Node( 
        package='rp3ba', executable='u2d2_node', name='U2D2', output='screen', 
        parameters=[{'delay_time': DELAY_TIME}], remappings=[ ('joints_goal', 'qd'), ('joints_state', 'qm') ]  )

    return LaunchDescription( [ 
        trajectory_node, 
        twist_mean_node, 
        inv_k_node, 
        u2d2_node ] )
