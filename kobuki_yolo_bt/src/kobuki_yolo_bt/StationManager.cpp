#include "kobuki_yolo_bt/StationManager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace kobuki_yolo_bt
{

using std::placeholders::_1;

StationManager::StationManager()
: Node("station_manager")
{
    // Cargar configuración de estaciones
    load_stations_config();
    
    // Suscripciones
    station_request_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/station_request", 10,
        std::bind(&StationManager::station_request_callback, this, _1));
        
    // Publicadores
    station_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/station_pose", 10);
    station_status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/station_status", 10);
    
    RCLCPP_INFO(this->get_logger(), "Station Manager initialized");
}

void StationManager::load_stations_config()
{
    std::string config_path = "config/stations.yaml";
    try
    {
        YAML::Node config = YAML::LoadFile(config_path);
        
        for (const auto& station : config["stations"])
        {
            StationInfo info;
            info.id = station["id"].as<std::string>();
            info.x = station["pose"]["x"].as<double>();
            info.y = station["pose"]["y"].as<double>();
            info.z = station["pose"]["z"].as<double>();
            info.qx = station["pose"]["qx"].as<double>();
            info.qy = station["pose"]["qy"].as<double>();
            info.qz = station["pose"]["qz"].as<double>();
            info.qw = station["pose"]["qw"].as<double>();
            
            stations_[info.id] = info;
            RCLCPP_INFO(this->get_logger(), "Loaded station: %s", info.id.c_str());
        }
    }
    catch (const YAML::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error loading stations config: %s", e.what());
    }
}

void StationManager::station_request_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string station_id = msg->data;
    
    if (stations_.find(station_id) == stations_.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Unknown station: %s", station_id.c_str());
        return;
    }
    
    const StationInfo& station = stations_[station_id];
    
    // Publicar la pose de la estación
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = this->now();
    pose_msg.pose.position.x = station.x;
    pose_msg.pose.position.y = station.y;
    pose_msg.pose.position.z = station.z;
    pose_msg.pose.orientation.x = station.qx;
    pose_msg.pose.orientation.y = station.qy;
    pose_msg.pose.orientation.z = station.qz;
    pose_msg.pose.orientation.w = station.qw;
    
    station_pose_pub_->publish(pose_msg);
    
    // Publicar el estado de la estación
    std_msgs::msg::String status_msg;
    status_msg.data = "station_" + station_id + "_pose_published";
    station_status_pub_->publish(status_msg);
}

} // namespace kobuki_yolo_bt 