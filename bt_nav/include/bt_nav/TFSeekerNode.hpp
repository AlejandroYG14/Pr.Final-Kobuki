#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "bt_nav/PIDController.hpp"

namespace tf_seeker
{

class TFSeekerNode : public rclcpp::Node
{
public:
  TFSeekerNode();
  void control_cycle();
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr get_vel_publisher() const { return vel_publisher_; }
  bool has_reached_goal() const { return goal_reached_; }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  tf_seeker::PIDController vlin_pid_;
  tf_seeker::PIDController vrot_pid_;

  bool goal_reached_ = false;


};

}  // namespace tf_seeker
