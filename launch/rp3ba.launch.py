from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

DELAY_TIME = 1/120

def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        'operation_mode', default_value='virtual',
        description='Operation mode for U2D2 node: "real" or "virtual"'
    )

    state_mode_arg = DeclareLaunchArgument(
        'robot_state_mode', default_value='position_only',
        description='State mode for robot state node: "position_only" or "full_state", full_state publish mobile base data and user_wrench'
    )

    application_arg = DeclareLaunchArgument(
        'PID', default_value='soft',
        description='PID sintonization: "soft" or "accuracy"'
    )

    kp_tbl = PythonExpression([ '2500 if \"', LaunchConfiguration('PID'), '\" == \"accuracy\" else 200' ])
    ki_tbl = PythonExpression([ '300 if \"', LaunchConfiguration('PID'), '\" == \"accuracy\" else 0' ])
    kd_tbl = PythonExpression([ '1 if \"', LaunchConfiguration('PID'), '\" == \"accuracy\" else 0' ])

    trajectory_node = Node(
        package='rp3ba', executable='trajectory_node', name='ref', output='screen',
        parameters=[{'delay_time': DELAY_TIME}], remappings=[ ('trayectoria', 'pd') ] )

    inv_kinematics_node = Node(
        package='rp3ba', executable='inv_kinematics_node.py', name='inv_kin', output='screen',
        remappings=[ ('angulos', 'qd') ,('pose', 'pd') ] )
        
    u2d2_robot_node = Node( 
        package='rp3ba', executable='u2d2_node', name='U2D2', output='screen', 
        parameters=[{'delay_time': DELAY_TIME, 'KP_TBL': kp_tbl, 'KI_TBL': ki_tbl, 'KD_TBL': kd_tbl}], 
        remappings=[ ('joints_goal', 'qd'), ('joints_state', 'qm') ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('operation_mode'), "' == 'real'"])) )
        
    u2d2_virtual_node = Node( 
        package='rp3ba', executable='u2d2_virtual_node.py', name='U2D2_virtual', output='screen', 
        remappings=[ ('joints_goal', 'qd'), ('joints_state', 'qm') ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('operation_mode'), "' == 'virtual'"])) )

    robot_state_node = Node(
        package='rp3ba', executable='robot_state_node.py', name='rob_state', 
        parameters=[{'state_mode': LaunchConfiguration('robot_state_mode')}], 
        output='screen', remappings=[ ('joint_data', 'qm'), ('joint_state', 'qs') , ('reference_state', 'qd') ] )
        
    visualization_data_node = Node( 
        package='rp3ba', executable='visualization_data_node.py', name='viz_data', output='screen', 
        parameters=[{'parent_frame': 'World_Link'},{'child_frame': 'MB_Link'}], 
        remappings=[ ('joint_state', 'qs'), ('joint_state_viz', 'qsv')]  )
        
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
        package='robot_state_publisher', executable='robot_state_publisher', name='rsp_arms',
        output='screen', parameters=[{'robot_description': robot_desc_arms}], arguments=[urdf_arms], 
        remappings=[ ('joint_states', 'qsv'), ('robot_description','robot_description_arms') ]  )

    robot_state_publisher_mobile_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='rsp_mobile',
        output='screen', parameters=[{'robot_description': robot_desc_mobile}], arguments=[urdf_mobile], 
        remappings=[ ('joint_states', 'joint_states_mobile'), ('robot_description','robot_description_mobile') ] )

    return LaunchDescription( [ 
        mode_arg,
        state_mode_arg,
        application_arg,
        trajectory_node, 
        inv_kinematics_node,
        u2d2_robot_node,
        u2d2_virtual_node,
        robot_state_node,
        visualization_data_node,
        rviz_node,
        robot_state_publisher_arms_node,
        robot_state_publisher_mobile_node] )


