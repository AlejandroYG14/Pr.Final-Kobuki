cmake_minimum_required(VERSION 3.8)
project(bt_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    nav2_msgs
    geometry_msgs
    tf2_geometry_msgs
    tf2
    tf2_ros
    vision_msgs
    behaviortree_cpp_v3
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

# bt_nav executables
add_executable(navigate src/bt_nav_main.cpp
  src/bt_nav/Move.cpp
  src/bt_nav/GetWaypoint.cpp
  src/bt_nav/GetOrigin.cpp
  src/bt_nav/Seeker.cpp
)
ament_target_dependencies(navigate ${dependencies})
target_link_libraries(navigate tf_seeker)

# tf_seeker components
add_library(tf_seeker SHARED
  src/bt_nav/TFPublisherNode.cpp
  src/bt_nav/TFSeekerNode.cpp
  src/bt_nav/PIDController.cpp
)
ament_target_dependencies(tf_seeker ${dependencies})

add_executable(tf_publisher src/tf_publisher.cpp)
ament_target_dependencies(tf_publisher ${dependencies})
target_link_libraries(tf_publisher tf_seeker)

add_executable(seeker src/seeker.cpp)
ament_target_dependencies(seeker ${dependencies})
target_link_libraries(seeker tf_seeker)

# Install targets
install(TARGETS
  navigate
  tf_seeker
  tf_publisher
  seeker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

# Install resource files
install(DIRECTORY
  behavior_tree_xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()

