#include <string>
#include <memory>
#include <chrono>

#include "bt_nav/Seeker.hpp"
#include "bt_nav/TFSeekerNode.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

namespace bt_nav
{

using namespace std::chrono_literals;

Seeker::Seeker(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  seek_duration_(20s)  // Buscará durante un máximo de 10 segundos
{
  config().blackboard->get("node", bt_node_);
  seeker_node_ = std::make_shared<tf_seeker::TFSeekerNode>();
}

void
Seeker::halt()
{
  geometry_msgs::msg::Twist stop_cmd;
  seeker_node_->get_vel_publisher()->publish(stop_cmd);
}

BT::NodeStatus
Seeker::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_INFO(bt_node_->get_logger(), "Iniciando búsqueda con Seeker...");
    start_time_ = bt_node_->now();
  }

  // Procesar el nodo ROS
  rclcpp::spin_some(seeker_node_);

  // Si ya alcanzó la meta, devolvemos SUCCESS
  if (seeker_node_->has_reached_goal()) {
    RCLCPP_INFO(bt_node_->get_logger(), "Persona alcanzada. Finalizando nodo Seeker.");
    return BT::NodeStatus::SUCCESS;
  }

  // Verificar si se ha agotado el tiempo de búsqueda
  auto elapsed = bt_node_->now() - start_time_;
  if (elapsed > seek_duration_) {
    RCLCPP_WARN(bt_node_->get_logger(), "Tiempo de búsqueda agotado. Abortando.");
    geometry_msgs::msg::Twist stop_cmd;
    seeker_node_->get_vel_publisher()->publish(stop_cmd);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace bt_nav

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav::Seeker>("Seeker");
}
