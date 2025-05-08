#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

#include "bt_nav/TFSeekerNode.hpp"

namespace tf_seeker
{

using namespace std::chrono_literals;

TFSeekerNode::TFSeekerNode()
: Node("tf_seeker"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  vlin_pid_(0.0, 1.0, 0.0, 0.7),
  vrot_pid_(0.0, 1.0, 0.3, 1.0),
  received_bool_(false)  // Inicializamos la variable recibida en false
{
  // Suscriptor al tópico 'flag_topic' para recibir el valor booleano
  subscription_bool_ = this->create_subscription<std_msgs::msg::Bool>(
    "flag_topic", 10,
    std::bind(&TFSeekerNode::callback, this, std::placeholders::_1)
  );

  // Publicador para enviar velocidades de control
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer para el ciclo de control, se ejecuta cada 50ms
  timer_ = create_wall_timer(
    50ms, std::bind(&TFSeekerNode::control_cycle, this));
}

// Callback para manejar el valor booleano recibido
void TFSeekerNode::callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // Almacena el valor recibido en la variable miembro
  received_bool_ = msg->data;
  RCLCPP_INFO(get_logger(), "Recibido: %s", received_bool_ ? "true" : "false");
}

void TFSeekerNode::control_cycle()
{
  // Solo actuamos si received_bool_ es verdadero
  if (received_bool_)
  {
    tf2::Stamped<tf2::Transform> bf2target;
    std::string error;

    // Comprobamos si es posible realizar la transformación
    if (tf_buffer_.canTransform("camera_rgb_optical_frame", "person", tf2::TimePointZero, &error)) {
      auto bf2target_msg = tf_buffer_.lookupTransform(
        "camera_rgb_optical_frame", "person", tf2::TimePointZero);

      tf2::fromMsg(bf2target_msg, bf2target);

      double x = bf2target.getOrigin().x();
      double y = bf2target.getOrigin().y();

      RCLCPP_INFO(get_logger(), "Ha enviado una velocidad: x = %.2f, y = %.2f", x, y);

      double angle = atan2(y, x);
      double dist = sqrt(x * x + y * y);

      // Control de velocidad usando el PID
      double vel_rot = std::clamp(vrot_pid_.get_output(angle), -0.5, 0.5);
      double vel_lin = std::clamp(vlin_pid_.get_output(dist-1), -0.3, 0.3);

      // Crear el mensaje Twist para publicar la velocidad
      geometry_msgs::msg::Twist twist;
      twist.linear.x = vel_lin;
      twist.angular.z = vel_rot;

      // Publicar el mensaje de velocidad
      vel_publisher_->publish(twist);

    } else {
      RCLCPP_WARN_STREAM(get_logger(), "Error en TF odom -> base_footprint [<< " << error << "]");
    }
  }
}

}  // namespace tf_seeker
