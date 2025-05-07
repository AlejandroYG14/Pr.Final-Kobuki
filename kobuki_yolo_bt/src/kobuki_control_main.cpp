#include "kobuki_yolo_bt/KobukiControlNode.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<kobuki_yolo_bt::KobukiControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 