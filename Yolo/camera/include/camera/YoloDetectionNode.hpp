#ifndef CAMERA__YOLO_DETECTION_NODE_HPP_
#define CAMERA__YOLO_DETECTION_NODE_HPP_

#include <memory>

#include "yolo_msgs/msg/detection_array.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camera
{

class YoloDetectionNode : public rclcpp::Node
{
public:
  YoloDetectionNode();

private:
  void detection_callback(const yolo_msgs::msg::DetectionArray::ConstSharedPtr & msg);

  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
};

}  // namespace camera

#endif  // CAMERA__YOLO_DETECTION_NODE_HPP_
