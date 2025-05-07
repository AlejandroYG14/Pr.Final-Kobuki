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

#ifndef KOBUKI_YOLO_BT__BEHAVIORTREENODE_HPP_
#define KOBUKI_YOLO_BT__BEHAVIORTREENODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace kobuki_yolo_bt
{

class BehaviorTreeNode : public rclcpp::Node
{
public:
  BehaviorTreeNode();
  
private:
  // Callbacks
  void voice_command_callback(const std_msgs::msg::String::ConstSharedPtr & msg);
  void robot_status_callback(const std_msgs::msg::String::ConstSharedPtr & msg);
  void object_detection_callback(const std_msgs::msg::String::ConstSharedPtr & msg);
  void object_picked_callback(const std_msgs::msg::String::ConstSharedPtr & msg);
  void object_placed_callback(const std_msgs::msg::String::ConstSharedPtr & msg);
  void station_status_callback(const std_msgs::msg::String::ConstSharedPtr & msg);
  
  // Behavior Tree
  void initialize_behavior_tree();
  void create_custom_nodes();
  void tick_behavior_tree();
  
  // Suscripciones
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_detection_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_picked_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_placed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr station_status_sub_;
  
  // Publicadores
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_object_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_station_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_detection_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Behavior Tree
  std::unique_ptr<BT::Tree> bt_;
  BT::BehaviorTreeFactory bt_factory_;
  
  // Blackboard para compartir información entre nodos
  BT::Blackboard::Ptr blackboard_;
  
  // Estado
  std::string requested_object_;
  std::string detected_object_;
  std::string current_robot_status_;
  std::string current_station_status_;
  bool robot_has_object_;
  int current_station_;
};

// Nodos personalizados de Behavior Tree

// Action Nodes
class FindObject : public BT::SyncActionNode
{
public:
  FindObject(const std::string & name, const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class GoToObject : public BT::SyncActionNode
{
public:
  GoToObject(const std::string & name, const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class PickObject : public BT::SyncActionNode
{
public:
  PickObject(const std::string & name, const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class GoToStation : public BT::SyncActionNode
{
public:
  GoToStation(const std::string & name, const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class PlaceObject : public BT::SyncActionNode
{
public:
  PlaceObject(const std::string & name, const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

// Condition Nodes
class ObjectDetected : public BT::ConditionNode
{
public:
  ObjectDetected(const std::string & name, const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class ObjectPicked : public BT::ConditionNode
{
public:
  ObjectPicked(const std::string & name, const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class AtStation : public BT::ConditionNode
{
public:
  AtStation(const std::string & name, const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class ObjectPlaced : public BT::ConditionNode
{
public:
  ObjectPlaced(const std::string & name, const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

}  // namespace kobuki_yolo_bt

#endif  // KOBUKI_YOLO_BT__BEHAVIORTREENODE_HPP_ 