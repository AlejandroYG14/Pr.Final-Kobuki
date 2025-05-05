// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KOBUKI_YOLO_BT__OBJECTDETECTIONNODE_HPP_
#define KOBUKI_YOLO_BT__OBJECTDETECTIONNODE_HPP_

#include <memory>
#include <string>
#include <vector>

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

class ObjectDetectionNode : public rclcpp::Node
{
public:
  ObjectDetectionNode();

private:
  // Callbacks
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void yolo_detection_callback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg);
  void target_object_callback(const std_msgs::msg::String::ConstSharedPtr & msg);
  
  // Procesar detectiones y encontrar objetos 3D
  void process_detections();
  geometry_msgs::msg::Pose calculate_3d_position(const vision_msgs::msg::Detection2D & detection);
  void publish_object_tf(const std::string & object_id, const geometry_msgs::msg::Pose & pose);
  
  // Suscripciones
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_object_sub_;
  
  // Publicadores
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection3d_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
  
  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Estado actual
  std::string target_object_;
  sensor_msgs::msg::Image::ConstSharedPtr current_image_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr current_pointcloud_;
  vision_msgs::msg::Detection2DArray::ConstSharedPtr current_detections_;
  std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> detected_objects_;
  
  // Parámetros
  double detection_threshold_;
  bool publish_debug_images_;
};

}  // namespace kobuki_yolo_bt

#endif  // KOBUKI_YOLO_BT__OBJECTDETECTIONNODE_HPP_ 