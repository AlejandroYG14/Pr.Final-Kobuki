#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "tf_seeker/TFPublisherNode.hpp"

namespace tf_seeker
{

using namespace std::chrono_literals;

TFPublisherNode::TFPublisherNode()
: Node("tf_producer"),
  transform_initialized_(false)
{

  std::cout << "Ingrese la clase objetivo (por ejemplo, bottle): ";
  std::cin >> target_class_;

  publisher_bool_ = this->create_publisher<std_msgs::msg::Bool>("flag_topic", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // Se suscribe al topic "detection_3d"
  detection_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "detections_3d", 10,
    std::bind(&TFPublisherNode::detection_callback, this, std::placeholders::_1));

    

    auto bool_msg = std_msgs::msg::Bool();
    bool_msg.data = true; 
    publisher_bool_->publish(bool_msg);

  // Publica la transformada cada 50ms
  timer_publish_ = create_wall_timer(
    50ms, std::bind(&TFPublisherNode::publish_tf, this));
}

void TFPublisherNode::detection_callback(
  const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  if (msg->detections.empty()) {
    RCLCPP_WARN(get_logger(), "No detections received");
    return;
  }

  

  for (const auto& det : msg->detections) {
    if (!det.results.empty() && det.results[0].hypothesis.class_id == target_class_) {

      // Actualiza el timestamp y el frame
      transform_.header.frame_id = "camera_rgb_optical_frame";  
      transform_.header.stamp = now();
      transform_.child_frame_id = "person";

      // Accede al 'bbox' desde 'Detection3D' y usa el 'center' para la posición
      const auto& bbox_center = det.bbox.center.position;
      transform_.transform.translation.x = bbox_center.z;  // x del centro del bbox
      transform_.transform.translation.y = -bbox_center.x;  // y del centro del bbox
      transform_.transform.translation.z = bbox_center.z;  // z del centro del bbox

      // Transformada inicializada
      transform_initialized_ = true;
      return;  // Solo procesamos la primera detección "person"
    }
  }

  RCLCPP_INFO(get_logger(), "No 'person' class detected in current message.");
}



void TFPublisherNode::publish_tf()
{
  if (!transform_initialized_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Transform not initialized yet, waiting for detection...");
    return;
  }

  // Publica la transformada "target" en tf
  transform_.header.stamp = now();  // Actualiza el timestamp
  tf_broadcaster_->sendTransform(transform_);
}

}  // namespace tf_seeker
