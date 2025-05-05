cmake_minimum_required(VERSION 3.8)
project(kobuki_yolo_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(pocketsphinx REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  vision_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  kobuki_ros_interfaces
  yolo_msgs
  cv_bridge
  image_transport
  yaml_cpp_vendor
  audio_common_msgs
  pocketsphinx
)

include_directories(include)

# Build the library with all the nodes
add_library(${PROJECT_NAME} SHARED
  src/kobuki_yolo_bt/VoiceCommandNode.cpp
  src/kobuki_yolo_bt/ObjectDetectionNode.cpp
  src/kobuki_yolo_bt/BehaviorTreeNode.cpp
  src/kobuki_yolo_bt/KobukiControlNode.cpp
  src/kobuki_yolo_bt/StationManager.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Ejecutables principales
add_executable(voice_command_node src/voice_command_main.cpp)
ament_target_dependencies(voice_command_node ${dependencies})
target_link_libraries(voice_command_node ${PROJECT_NAME})

add_executable(object_detection_node src/object_detection_main.cpp)
ament_target_dependencies(object_detection_node ${dependencies})
target_link_libraries(object_detection_node ${PROJECT_NAME})

add_executable(behavior_tree_node src/behavior_tree_main.cpp)
ament_target_dependencies(behavior_tree_node ${dependencies})
target_link_libraries(behavior_tree_node ${PROJECT_NAME})

add_executable(kobuki_control_node src/kobuki_control_main.cpp)
ament_target_dependencies(kobuki_control_node ${dependencies})
target_link_libraries(kobuki_control_node ${PROJECT_NAME})

add_executable(station_manager_node src/station_manager_main.cpp)
ament_target_dependencies(station_manager_node ${dependencies})
target_link_libraries(station_manager_node ${PROJECT_NAME})

install(TARGETS 
  ${PROJECT_NAME}
  voice_command_node
  object_detection_node
  behavior_tree_node
  kobuki_control_node
  station_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 