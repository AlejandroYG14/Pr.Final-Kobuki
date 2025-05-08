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
#include <iostream>

#include "bt_nav/GetOrigin.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace bt_nav
{

GetOrigin::GetOrigin(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  origin_.header.frame_id = "map";
  origin_.pose.orientation.w = 1.0;

  // Coordenadas del origen (0,0)
  origin_.pose.position.x = 0.65;
  origin_.pose.position.y = -8.63;
}

void
GetOrigin::halt()
{
}

BT::NodeStatus
GetOrigin::tick()
{
  setOutput("origin", origin_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nav


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav::GetOrigin>("GetOrigin");
} 