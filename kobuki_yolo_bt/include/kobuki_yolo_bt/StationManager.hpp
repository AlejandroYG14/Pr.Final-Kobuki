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

#ifndef KOBUKI_YOLO_BT_STATION_MANAGER_HPP_
#define KOBUKI_YOLO_BT_STATION_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <map>
#include <string>

namespace kobuki_yolo_bt
{

struct StationInfo
{
    std::string id;
    double x, y, z;
    double qx, qy, qz, qw;
};

class StationManager : public rclcpp::Node
{
public:
    StationManager();

private:
    void load_stations_config();
    void station_request_callback(const std_msgs::msg::String::SharedPtr msg);
    
    // Suscripciones
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr station_request_sub_;
    
    // Publicadores
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr station_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr station_status_pub_;
    
    // Datos
    std::map<std::string, StationInfo> stations_;
};

} // namespace kobuki_yolo_bt

#endif // KOBUKI_YOLO_BT_STATION_MANAGER_HPP_ 