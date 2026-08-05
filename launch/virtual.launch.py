from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

DELAY_TIME = 1/120

def generate_launch_description():

    trajectory_node = Node(
        package='rp3ba', executable='trajectory_node', name='Ref', output='screen',
        parameters=[{'delay_time': DELAY_TIME}], remappings=[ ('trayectoria', 'pd') ] )

    inv_kinematics_node = Node(
        package='rp3ba', executable='inv_kinematics_node.py', name='Inv_k', output='screen',
        remappings=[ ('angulos', 'qd') ,('pose', 'pd') ] )
        
    u2d2_virtual_node = Node( 
        package='rp3ba', executable='u2d2_virtual_node.py', name='U2D2_virtual', output='screen', 
        parameters=[{'delay_time': DELAY_TIME},{'parent_frame': 'World_Link'},{'child_frame': 'MB_Link'}], remappings=[ ('joints_goal', 'qd'), ('joints_state', 'qm') , ('pose', 'pd') ]  )
        
    rviz_config_path = join( get_package_share_directory("rp3ba"), 'rviz', 'config.rviz' )
        
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config_path] )

    urdf_arms = join( get_package_share_directory('rp3ba'), 'urdf', 'arms.urdf')
    with open(urdf_arms, 'r') as infp_arms:
        robot_desc_arms = infp_arms.read()
        
    urdf_mobile = join( get_package_share_directory('rp3ba'), 'urdf', 'mobile.urdf')
    with open(urdf_mobile, 'r') as infp_mobile:
        robot_desc_mobile = infp_mobile.read()
        
    robot_state_publisher_arms_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher_arms',
        output='screen', parameters=[{'robot_description': robot_desc_arms}], arguments=[urdf_arms], 
        remappings=[ ('joint_states', 'qm'), ('robot_description','robot_description_arms') ]  )

    robot_state_publisher_mobile_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher_mobile',
        output='screen', parameters=[{'robot_description': robot_desc_mobile}], arguments=[urdf_mobile], 
        remappings=[ ('joint_states', 'joint_states_mobile'), ('robot_description','robot_description_mobile') ] )

    return LaunchDescription( [ 
        trajectory_node, 
        inv_kinematics_node,
        u2d2_virtual_node,
        rviz_node,
        robot_state_publisher_arms_node,
        robot_state_publisher_mobile_node] )
