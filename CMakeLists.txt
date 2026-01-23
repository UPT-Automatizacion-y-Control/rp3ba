cmake_minimum_required(VERSION 3.8)
project(rp3ba)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(filters REQUIRED)

include_directories(include)

add_executable(trayectoria_r src/trayectoria_r.cpp)
ament_target_dependencies(trayectoria_r rclcpp std_msgs geometry_msgs)
target_link_libraries(trayectoria_r "${cpp_typesupport_target}")

add_executable(twist_mean_filter_node src/twist_mean_filter_node.cpp)
ament_target_dependencies(twist_mean_filter_node rclcpp geometry_msgs filters)
target_link_libraries(twist_mean_filter_node "${cpp_typesupport_target}")

add_executable(u2d2_node src/u2d2_node.cpp)
ament_target_dependencies(u2d2_node rclcpp sensor_msgs dynamixel_sdk)
target_link_libraries(u2d2_node "${cpp_typesupport_target}")

install(PROGRAMS
  scripts/inv_kinematics.py
  scripts/twistPose2TF.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  trayectoria_r
  u2d2_node
  twist_mean_filter_node
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
