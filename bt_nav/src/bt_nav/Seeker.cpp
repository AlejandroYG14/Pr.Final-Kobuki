// Copyright 2021 Intelligent Robotics Lab
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

#include <string>
#include <memory>
#include <chrono>

#include "bt_nav/Seeker.hpp"
#include "bt_nav/TFSeekerNode.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

namespace bt_nav
{

using namespace std::chrono_literals;

Seeker::Seeker(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  seek_duration_(10s)  // Por defecto buscará durante 10 segundos
{
  config().blackboard->get("node", bt_node_);
  seeker_node_ = std::make_shared<tf_seeker::TFSeekerNode>();
}

void
Seeker::halt()
{
  // Detener la búsqueda si es necesario
  geometry_msgs::msg::Twist stop_cmd;
  seeker_node_->get_vel_publisher()->publish(stop_cmd);
}

BT::NodeStatus
Seeker::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    // Primera vez que se ejecuta este nodo
    RCLCPP_INFO(bt_node_->get_logger(), "Iniciando búsqueda con Seeker...");
    start_time_ = bt_node_->now();
    
    // Activar el nodo seeker
    std_msgs::msg::Bool msg;
    msg.data = true;
    seeker_node_->callback(std::make_shared<std_msgs::msg::Bool>(msg));
  }

  // Comprobar si ha pasado el tiempo de búsqueda
  auto elapsed = bt_node_->now() - start_time_;
  if (elapsed > seek_duration_) {
    RCLCPP_INFO(bt_node_->get_logger(), "Búsqueda completada después de %d segundos", 
                static_cast<int>(seek_duration_.count()));
    
    // Desactivar el nodo seeker
    std_msgs::msg::Bool msg;
    msg.data = false;
    seeker_node_->callback(std::make_shared<std_msgs::msg::Bool>(msg));
    
    // Detener el robot
    geometry_msgs::msg::Twist stop_cmd;
    seeker_node_->get_vel_publisher()->publish(stop_cmd);
    
    return BT::NodeStatus::SUCCESS;
  }

  // Procesar eventos
  rclcpp::spin_some(seeker_node_);
  
  // Todavía estamos buscando
  return BT::NodeStatus::RUNNING;
}

}  // namespace bt_nav

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav::Seeker>("Seeker");
} 