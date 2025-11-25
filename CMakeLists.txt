cmake_minimum_required(VERSION 3.8)
project(ros2_mobile_manipulator_full)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xacro REQUIRED)

add_executable(mission_control_node src/mission_control_node.cpp)
ament_target_dependencies(mission_control_node
  rclcpp rclcpp_action std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs nav2_msgs tf2_ros
)

add_executable(mock_arm_node src/mock_arm_node.cpp)
ament_target_dependencies(mock_arm_node
  rclcpp std_srvs std_msgs
)

install(TARGETS
  mission_control_node
  mock_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config .github docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

# MoveIt pick/place node
add_executable(pick_place_moveit src/pick_place_moveit.cpp)
ament_target_dependencies(pick_place_moveit rclcpp moveit_ros_planning_interface)
install(TARGETS pick_place_moveit DESTINATION lib/${PROJECT_NAME})
