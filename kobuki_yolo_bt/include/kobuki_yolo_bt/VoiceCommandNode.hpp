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

#ifndef KOBUKI_YOLO_BT__VOICECOMMANDNODE_HPP_
#define KOBUKI_YOLO_BT__VOICECOMMANDNODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "audio_common_msgs/msg/audio_data.hpp"

namespace kobuki_yolo_bt
{

class VoiceCommandNode : public rclcpp::Node
{
public:
  VoiceCommandNode();

private:
  void audio_callback(const audio_common_msgs::msg::AudioData::ConstSharedPtr & msg);
  void process_command(const std::string & command);

  // Subscribes a datos de audio
  rclcpp::Subscription<audio_common_msgs::msg::AudioData>::SharedPtr audio_sub_;
  
  // Publica comandos reconocidos
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  
  // Palabras clave reconocibles (botella, pelota, etc.)
  std::vector<std::string> recognized_objects_;
  
  // Comandos reconocibles (buscar, ir a, etc.)
  std::vector<std::string> recognized_commands_;
};

}  // namespace kobuki_yolo_bt

#endif  // KOBUKI_YOLO_BT__VOICECOMMANDNODE_HPP_ 