#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "std_msgs/msg/bool.hpp"


namespace tf_seeker
{

class TFPublisherNode : public rclcpp::Node
{
public:
  TFPublisherNode();

private:
  void detection_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
  void publish_tf();

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
  rclcpp::TimerBase::SharedPtr timer_publish_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_bool_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_;

  bool transform_initialized_;
  std::string target_class_;
};

}  // namespace tf_seeker
