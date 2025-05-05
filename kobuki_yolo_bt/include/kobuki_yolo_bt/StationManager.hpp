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

#ifndef KOBUKI_YOLO_BT__STATIONMANAGER_HPP_
#define KOBUKI_YOLO_BT__STATIONMANAGER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

namespace kobuki_yolo_bt
{

struct Station
{
  int id;
  std::string name;
  geometry_msgs::msg::Pose pose;
  std::vector<std::string> allowed_objects;
  bool has_object;
  std::string current_object;
};

class StationManager : public rclcpp::Node
{
public:
  StationManager();

private:
  // Callbacks
  void goto_station_callback(const std_msgs::msg::Int32::ConstSharedPtr & msg);
  void object_detected_callback(const std_msgs::msg::String::ConstSharedPtr & msg);
  void object_picked_callback(const std_msgs::msg::String::ConstSharedPtr & msg);
  void object_placed_callback(const std_msgs::msg::String::ConstSharedPtr & msg);
  
  // Métodos para manejo de estaciones
  void initialize_stations();
  void publish_station_tfs();
  bool is_object_allowed_at_station(const std::string & object, int station_id);
  
  // Timer callback
  void timer_callback();
  
  // Suscripciones
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goto_station_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_picked_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_placed_sub_;
  
  // Publicadores
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr station_status_pub_;
  
  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Estado de estaciones
  std::vector<Station> stations_;
  int current_station_id_;
  
  // Estado del robot
  bool robot_has_object_;
  std::string robot_object_;
};

}  // namespace kobuki_yolo_bt

#endif  // KOBUKI_YOLO_BT__STATIONMANAGER_HPP_ 