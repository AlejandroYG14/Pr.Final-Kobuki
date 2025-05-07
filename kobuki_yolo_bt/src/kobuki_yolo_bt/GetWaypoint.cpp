#include "kobuki_yolo_bt/GetWaypoint.hpp"

#include <string>
#include <iostream>
#include <vector>
#include <chrono>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kobuki_yolo_bt
{

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), current_station_index_(0)
{
    // Obtener nodo desde blackboard
    rclcpp::Node::SharedPtr node;
    config().blackboard->get("node", node);

    // Inicializar estaciones
    // Estación 1 - Al lado de la puerta
    geometry_msgs::msg::PoseStamped station1_pose;
    station1_pose.header.frame_id = "map";
    station1_pose.pose.position.x = 3.7830708026885986;
    station1_pose.pose.position.y = -9.048851013183594;
    station1_pose.pose.position.z = -0.001373291015625;
    station1_pose.pose.orientation.w = 1.0;
    stations_.push_back({"station1", station1_pose});

    // Estación 2 - Al lado de la mesa grande
    geometry_msgs::msg::PoseStamped station2_pose;
    station2_pose.header.frame_id = "map";
    station2_pose.pose.position.x = 4.615912914276123;
    station2_pose.pose.position.y = -4.381537914276123;
    station2_pose.pose.position.z = -0.0013427734375;
    station2_pose.pose.orientation.w = 1.0;
    stations_.push_back({"station2", station2_pose});

    // Inicializar posición actual
    wp_ = stations_[current_station_index_].second;
    last_station_change_ = std::chrono::steady_clock::now();
}

void
GetWaypoint::halt()
{
    // Nada que hacer al detener el nodo
}

BT::NodeStatus
GetWaypoint::tick()
{
    std::string station_id;
    if (getInput("station_id", station_id)) {
        // Si se proporciona un ID de estación específico, obtener esa estación
        wp_ = getStationPose(station_id);
    } else {
        // Comprobar si ha pasado el tiempo de timeout desde el último cambio de estación
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_station_change_);
        
        if (elapsed > timeout_) {
            // Si ha pasado el timeout, cambiar a la siguiente estación
            current_station_index_ = (current_station_index_ + 1) % stations_.size();
            wp_ = stations_[current_station_index_].second;
            last_station_change_ = now;
            
            RCLCPP_INFO(rclcpp::get_logger("GetWaypoint"), 
                     "Timeout en estación actual. Cambiando a estación: %s", 
                     stations_[current_station_index_].first.c_str());
        } else {
            // Mantener la estación actual
            wp_ = stations_[current_station_index_].second;
        }
    }

    // Establecer la salida
    setOutput("waypoint", wp_);
    return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::PoseStamped 
GetWaypoint::getStationPose(const std::string& station_id) 
{
    for (const auto& station : stations_) {
        if (station.first == station_id) {
            return station.second;
        }
    }
    
    // Si no se encuentra, devolver la primera estación
    RCLCPP_WARN(rclcpp::get_logger("GetWaypoint"), 
             "Estación %s no encontrada. Usando primera estación.", 
             station_id.c_str());
    return stations_[0].second;
}

}  // namespace kobuki_yolo_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<kobuki_yolo_bt::GetWaypoint>("GetWaypoint");
} 