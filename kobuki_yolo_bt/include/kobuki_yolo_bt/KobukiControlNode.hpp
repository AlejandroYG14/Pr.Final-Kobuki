// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KOBUKI_YOLO_BT__KOBUKICONTROLNODE_HPP_
#define KOBUKI_YOLO_BT__KOBUKICONTROLNODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace kobuki_yolo_bt
{

enum class RobotState
{
  IDLE,
  NAVIGATING_TO_OBJECT,
  APPROACHING_OBJECT,
  PICKING_OBJECT,
  NAVIGATING_TO_STATION,
  APPROACHING_STATION,
  PLACING_OBJECT
};

class KobukiControlNode : public rclcpp::Node
{
public:
  KobukiControlNode();

private:
  // Callbacks
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void bumper_callback(const kobuki_ros_interfaces::msg::BumperEvent::ConstSharedPtr & msg);
  void target_object_callback(const std_msgs::msg::String::ConstSharedPtr & msg);
  void target_station_callback(const std_msgs::msg::Int32::ConstSharedPtr & msg);
  
  // Control
  void control_cycle();
  void navigate_to_frame(const std::string & target_frame);
  bool approach_frame(const std::string & target_frame, double stop_distance);
  bool pick_object();
  bool place_object();
  void stop_robot();
  
  // Suscripciones
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_object_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_station_sub_;
  
  // Publicadores
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr picked_object_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr placed_object_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr object_request_pub_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Estado
  RobotState state_;
  std::string target_object_;
  int target_station_;
  bool bumper_pressed_;
  bool has_object_;
  std::string current_object_;
  
  // Parámetros de control
  double linear_velocity_;
  double angular_velocity_;
  double approach_distance_;
  double pickup_distance_;
  double placement_distance_;
  double position_tolerance_;
  double orientation_tolerance_;
};

}  // namespace kobuki_yolo_bt

#endif  // KOBUKI_YOLO_BT__KOBUKICONTROLNODE_HPP_ 