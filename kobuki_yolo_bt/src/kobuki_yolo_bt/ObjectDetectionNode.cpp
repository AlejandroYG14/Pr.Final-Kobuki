#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "kobuki_yolo_bt/ObjectDetectionNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "cv_bridge/cv_bridge.h"

namespace kobuki_yolo_bt
{

using std::placeholders::_1;

ObjectDetectionNode::ObjectDetectionNode()
: Node("object_detection_node"),
  target_object_(""),
  detection_threshold_(0.7),
  publish_debug_images_(true)
{
  // Declarar parámetros
  declare_parameter("detection_threshold", 0.7);
  declare_parameter("publish_debug_images", true);
  
  // Leer parámetros
  detection_threshold_ = get_parameter("detection_threshold").as_double();
  publish_debug_images_ = get_parameter("publish_debug_images").as_bool();
  
  // Inicializar TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  
  // Crear suscripciones
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", 10, std::bind(&ObjectDetectionNode::image_callback, this, _1));
  
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/points", 10, std::bind(&ObjectDetectionNode::point_cloud_callback, this, _1));
  
  detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    "/yolo/detections", 10, std::bind(&ObjectDetectionNode::yolo_detection_callback, this, _1));
  
  target_object_sub_ = create_subscription<std_msgs::msg::String>(
    "/voice_command", 10, std::bind(&ObjectDetectionNode::target_object_callback, this, _1));
  
  // Crear publicadores
  detection3d_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
    "/object_detections_3d", 10);
  
  if (publish_debug_images_) {
    debug_img_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/detection_debug_image", 10);
  }
  
  RCLCPP_INFO(get_logger(), "Nodo de detección de objetos inicializado");
}

void ObjectDetectionNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  current_image_ = msg;
}

void ObjectDetectionNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  current_pointcloud_ = msg;
}

void ObjectDetectionNode::yolo_detection_callback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg)
{
  current_detections_ = msg;
  
  // Si tenemos la imagen, la nube de puntos y las detecciones, procesamos
  if (current_image_ && current_pointcloud_ && current_detections_) {
    process_detections();
  }
}

void ObjectDetectionNode::target_object_callback(const std_msgs::msg::String::ConstSharedPtr & msg)
{
  target_object_ = msg->data;
  RCLCPP_INFO(get_logger(), "Nuevo objeto objetivo: %s", target_object_.c_str());
}

void ObjectDetectionNode::process_detections()
{
  detected_objects_.clear();
  
  // Preparar mensaje para publicar detecciones 3D
  vision_msgs::msg::Detection3DArray detection3d_array_msg;
  detection3d_array_msg.header = current_detections_->header;
  
  // Procesar cada detección 2D
  for (const auto & detection : current_detections_->detections) {
    // Extraer nombre de clase y score
    const auto & result = detection.results.front();
    std::string object_class = result.hypothesis.class_id;
    float score = result.hypothesis.score;
    
    // Verificar si cumple con el umbral y si es un objeto que nos interesa
    if (score >= detection_threshold_ && 
        (target_object_.empty() || object_class == target_object_)) {
      
      // Calcular la posición 3D
      geometry_msgs::msg::Pose pose = calculate_3d_position(detection);
      
      // Añadir a la lista de objetos detectados
      detected_objects_.push_back(std::make_pair(object_class, pose));
      
      // Publicar TF para el objeto
      std::string frame_id = object_class + "_" + std::to_string(detection.bbox.center.position.x);
      publish_object_tf(frame_id, pose);
      
      // Si tenemos un objeto objetivo específico y lo hemos encontrado, publicar info
      if (!target_object_.empty() && object_class == target_object_) {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = frame_id;
        detection3d_pub_->publish(detection3d_array_msg);
      }
    }
  }
  
  // Publicar todas las detecciones 3D
  detection3d_pub_->publish(detection3d_array_msg);
}

geometry_msgs::msg::Pose ObjectDetectionNode::calculate_3d_position(const vision_msgs::msg::Detection2D & detection)
{
  // En una implementación real, aquí se calcularía la posición 3D a partir de
  // la nube de puntos o información de profundidad.
  // Para esta implementación simplificada, sólo simulamos valores.
  
  geometry_msgs::msg::Pose pose;
  
  // Ubicamos el objeto frente al robot a una distancia arbitraria
  pose.position.x = 1.0;  // 1 metro delante del robot
  pose.position.y = 0.0;
  pose.position.z = 0.1;  // Ligeramente por encima del suelo
  
  // Orientación neutral (sin rotación)
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  
  return pose;
}

void ObjectDetectionNode::publish_object_tf(const std::string & object_id, const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::TransformStamped transform;
  
  transform.header.stamp = this->get_clock()->now();
  transform.header.frame_id = "base_link";  // Frame del robot
  transform.child_frame_id = object_id;
  
  // Copiar la posición y orientación
  transform.transform.translation.x = pose.position.x;
  transform.transform.translation.y = pose.position.y;
  transform.transform.translation.z = pose.position.z;
  
  transform.transform.rotation.x = pose.orientation.x;
  transform.transform.rotation.y = pose.orientation.y;
  transform.transform.rotation.z = pose.orientation.z;
  transform.transform.rotation.w = pose.orientation.w;
  
  // Publicar la transformación
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace kobuki_yolo_bt 