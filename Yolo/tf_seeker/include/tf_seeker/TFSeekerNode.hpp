
#ifndef TF_SEEKER__TFSEEKERNODE_HPP_
#define TF_SEEKER__TFSEEKERNODE_HPP_

#include "tf_seeker/PIDController.hpp"

#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace tf_seeker
{

class TFSeekerNode : public rclcpp::Node
{
public:
  TFSeekerNode();

private:
  void control_cycle();
  void callback(const std_msgs::msg::Bool::SharedPtr msg);


  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_bool_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  PIDController vlin_pid_, vrot_pid_;
  bool received_bool_;
};

}  //  namespace tf_seeker

#endif  // TF_SEEKER__TFSEEKERNODE_HPP_
