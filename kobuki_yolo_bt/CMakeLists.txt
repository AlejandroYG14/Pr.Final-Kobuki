cmake_minimum_required(VERSION 3.8)
project(kobuki_yolo_bt)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
)

add_executable(behavior_tree_node 
  src/behavior_tree_main.cpp 
  src/kobuki_yolo_bt/BehaviorTreeNode.cpp
  src/kobuki_yolo_bt/GetWaypoint.cpp
  src/kobuki_yolo_bt/NavigateToWaypoint.cpp
)
ament_target_dependencies(behavior_tree_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
  nav2_msgs
)

add_executable(kobuki_control_node src/kobuki_control_main.cpp src/kobuki_yolo_bt/KobukiControlNode.cpp)
ament_target_dependencies(kobuki_control_node
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(station_manager_node src/station_manager_main.cpp src/kobuki_yolo_bt/StationManager.cpp)
ament_target_dependencies(station_manager_node
  rclcpp
  std_msgs
  geometry_msgs
  yaml-cpp
)

target_link_libraries(station_manager_node
  yaml-cpp
)

# Añadir el nuevo ejecutable
add_executable(station_navigator_node src/station_navigator_node.cpp)
ament_target_dependencies(station_navigator_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2
  std_msgs)

install(TARGETS
  behavior_tree_node
  kobuki_control_node
  station_manager_node
  station_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()