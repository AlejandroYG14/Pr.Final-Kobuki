#include "kobuki_yolo_bt/StationManager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<kobuki_yolo_bt::StationManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 