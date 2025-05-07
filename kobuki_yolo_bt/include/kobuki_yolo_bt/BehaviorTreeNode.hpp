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

#ifndef KOBUKI_YOLO_BT__BEHAVIOR_TREE_NODE_HPP_
#define KOBUKI_YOLO_BT__BEHAVIOR_TREE_NODE_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace kobuki_yolo_bt
{

class BehaviorTreeNode : public BT::StatefulActionNode
{
public:
    BehaviorTreeNode(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<std::string>("object"),
            BT::OutputPort<std::string>("result")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void object_request_callback(const std_msgs::msg::String::SharedPtr msg);

    // Variables
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_request_sub_;
    bool object_request_received_ = false;
    std::string requested_object_;
};

} // namespace kobuki_yolo_bt

#endif // KOBUKI_YOLO_BT__BEHAVIOR_TREE_NODE_HPP_ 