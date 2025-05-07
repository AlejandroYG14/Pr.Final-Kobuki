#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "kobuki_yolo_bt/VoiceCommandNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kobuki_yolo_bt::VoiceCommandNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 