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

#ifndef BT_NAV__SEEKER_HPP_
#define BT_NAV__SEEKER_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "bt_nav/TFSeekerNode.hpp"

namespace bt_nav
{

class Seeker : public BT::ActionNodeBase
{
public:
  explicit Seeker(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() override;
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList();  // No necesita puertos de entrada o salida
  }

private:
  rclcpp::Node::SharedPtr bt_node_;
  std::shared_ptr<tf_seeker::TFSeekerNode> seeker_node_;
  rclcpp::Time start_time_;
  std::chrono::seconds seek_duration_;
};

}  // namespace bt_nav

#endif  // BT_NAV__SEEKER_HPP_ 