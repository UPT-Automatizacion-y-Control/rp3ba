from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

DELAY_TIME = 1/70

def generate_launch_description():

    urdf_arms = join( get_package_share_directory('rp3ba'), 'urdf', 'arms.urdf')
    with open(urdf_arms, 'r') as infp_arms:
        robot_desc_arms = infp_arms.read()
        
    urdf_mobile = join( get_package_share_directory('rp3ba'), 'urdf', 'mango.urdf')
    with open(urdf_mobile, 'r') as infp_mobile:
        robot_desc_mobile = infp_mobile.read()
        
    rviz_config_path = join( get_package_share_directory("rp3ba"), 'rviz', 'config.rviz' )

    robot_state_publisher_arms_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher_arms',
        output='screen', parameters=[{'robot_description': robot_desc_arms}], arguments=[urdf_arms], 
        remappings=[ ('joint_states', 'qd'), ('robot_description','robot_description_arms') ]  )

    robot_state_publisher_mobile_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher_mobile',
        output='screen', parameters=[{'robot_description': robot_desc_mobile}], arguments=[urdf_mobile], 
        remappings=[ ('joint_states', 'joint_states_mobile'), ('robot_description','robot_description_mobile') ] )
        
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config_path] )

    trajectory_node = Node(
        package='rp3ba', executable='trayectoria_r', name='ref', output='screen',
        parameters=[{'delay_time': DELAY_TIME}], remappings=[ ('trayectoria', 'pose') ] )

    twist_mean_node = Node(
        package='rp3ba', executable='twist_mean_filter_node', name='ref_mean', output='screen',
        remappings=[ ('twist_in', 'pose'), ('twist_out', 'pose_fil') ] )
        
    twist_pose_node = Node(
        package='rp3ba', executable='twistPose2TF.py', name='twistPose2TF', output='screen',
         parameters=[{'parent_frame': 'World_Link'},{'child_frame': 'MB_Link'}], remappings=[ ('pose_twist', 'pose_fil') ] )

    inv_k_node = Node(
        package='rp3ba', executable='inv_kinematics.py', name='Inv_k', output='screen',
        remappings=[ ('pose', 'pose_fil'), ('angulos', 'qd') ] )

    return LaunchDescription( [ 
        robot_state_publisher_arms_node,
        robot_state_publisher_mobile_node,
        rviz_node,
        trajectory_node, 
        twist_mean_node, 
        twist_pose_node, 
        inv_k_node ] )
