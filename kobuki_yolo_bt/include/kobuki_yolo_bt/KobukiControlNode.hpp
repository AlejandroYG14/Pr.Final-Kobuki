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

#ifndef KOBUKI_YOLO_BT_KOBUKI_CONTROL_NODE_HPP_
#define KOBUKI_YOLO_BT_KOBUKI_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

namespace kobuki_yolo_bt
{

class KobukiControlNode : public rclcpp::Node
{
public:
    KobukiControlNode();

private:
    void command_callback(const std_msgs::msg::String::SharedPtr msg);
    
    void stop_robot();
    void move_forward();
    void move_backward();
    void turn_left();
    void turn_right();
    void pick_object();
    
    // Suscripciones
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    
    // Publicadores
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

} // namespace kobuki_yolo_bt

#endif // KOBUKI_YOLO_BT_KOBUKI_CONTROL_NODE_HPP_ 