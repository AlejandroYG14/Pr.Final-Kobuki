#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "kobuki_yolo_bt/ObjectDetectionNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kobuki_yolo_bt::ObjectDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 