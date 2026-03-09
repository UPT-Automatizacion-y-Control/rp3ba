from launch import LaunchDescription
from launch_ros.actions import Node

DELAY_TIME = 1/120

def generate_launch_description():

    trajectory_node = Node(
        package='rp3ba', executable='trajectory_node', name='Ref', output='screen',
        parameters=[{'delay_time': DELAY_TIME}], remappings=[ ('trayectoria', 'pose') , ('platform_pose','pm')] )

    inv_kinematics_node = Node(
        package='rp3ba', executable='inv_kinematics_node.py', name='Inv_k', output='screen',
        remappings=[ ('angulos', 'qd') ] )
        
    u2d2_node = Node( 
        package='rp3ba', executable='u2d2_node', name='U2D2', output='screen', 
        parameters=[{'delay_time': DELAY_TIME}], remappings=[ ('joints_goal', 'qd'), ('joints_state', 'qm') ]  )

    platform_state_node = Node(
        package='rp3ba', executable='platform_state_node.py', name='P_state', output='screen',
        remappings=[  ('joints_state', 'qm'), ('platform_pose','pm') ] )

    return LaunchDescription( [ 
        trajectory_node, 
        inv_kinematics_node, 
        u2d2_node,
        platform_state_node] )
