#include "bt_nav/TFSeekerNode.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bt_nav/PIDController.hpp"


namespace tf_seeker
{

using namespace std::chrono_literals;

TFSeekerNode::TFSeekerNode()
: Node("tf_seeker"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  vlin_pid_(0.0, 1.0, 0.0, 0.7),
  vrot_pid_(0.0, 1.0, 0.3, 1.0)
{
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = create_wall_timer(
    50ms, std::bind(&TFSeekerNode::control_cycle, this));
}

void TFSeekerNode::control_cycle()
{
  tf2::Stamped<tf2::Transform> bf2target;
  std::string error;

  if (tf_buffer_.canTransform("camera_rgb_optical_frame", "person", tf2::TimePointZero, &error)) {
    auto bf2target_msg = tf_buffer_.lookupTransform(
      "camera_rgb_optical_frame", "person", tf2::TimePointZero);

    tf2::fromMsg(bf2target_msg, bf2target);

    double x = bf2target.getOrigin().x();
    double y = bf2target.getOrigin().y();

    double angle = atan2(y, x);
    double dist = sqrt(x * x + y * y);

    if (dist < 1.0) {
      goal_reached_ = true;

      // Detener el robot
      geometry_msgs::msg::Twist stop;
      vel_publisher_->publish(stop);
      const char* banner = R"(
 __   ___  ___  __   __  ___  ___  ___
|  _ \ |  __||_   _||  __| / __||_   _||  __||  _ \
| |  | || |_      | |   | |_   | |        | |   | |__   | |  | |
| |  | ||  _|     | |   |  _|  | |        | |   |  __|  | |  | |
| |_| || |_    | |   | |__ | |__    | |   | |__ | |__| |
|__/ |__|   ||   |__| \__|   ||   |__||___/
)";

  // Imprimir el banner con rclcpp_info sin colores
      RCLCPP_INFO(get_logger(), "%s", banner);
      return;
    } else {
      goal_reached_ = false;
    }

    double vel_rot = std::clamp(vrot_pid_.get_output(angle), -0.5, 0.5);
    double vel_lin = std::clamp(vlin_pid_.get_output(dist - 0.5), -0.3, 0.3);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = vel_lin;
    twist.angular.z = vel_rot;

    vel_publisher_->publish(twist);

    RCLCPP_INFO(get_logger(), "Distancia: %.2f m, x = %.2f, y = %.2f", dist, x, y);
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Error en TF: " << error);
  }
}

}  // namespace tf_seeker
