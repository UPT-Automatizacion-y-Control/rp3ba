cmake_minimum_required(VERSION 3.8)
project(rp3ba)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(trajectory_node src/trajectory_node.cpp)
ament_target_dependencies(trajectory_node rclcpp std_msgs geometry_msgs tf2 tf2_geometry_msgs)
target_link_libraries(trajectory_node "${cpp_typesupport_target}")

add_executable(u2d2_node src/u2d2_node.cpp)
ament_target_dependencies(u2d2_node rclcpp sensor_msgs dynamixel_sdk)
target_link_libraries(u2d2_node "${cpp_typesupport_target}")

install(TARGETS
  trajectory_node
  u2d2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/inv_kinematics_node.py
  scripts/platform_state_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  meshes
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
