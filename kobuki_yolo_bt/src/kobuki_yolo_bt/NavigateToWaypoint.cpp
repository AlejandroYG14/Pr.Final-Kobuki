#include "kobuki_yolo_bt/NavigateToWaypoint.hpp"

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace kobuki_yolo_bt
{

NavigateToWaypoint::NavigateToWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  navigation_succeeded_(false),
  navigation_active_(false),
  navigation_aborted_(false)
{
    // Obtener nodo desde blackboard
    config().blackboard->get("node", node_);

    // Crear cliente de acción para navegación
    nav_action_client_ = rclcpp_action::create_client<NavigateAction>(
        node_, "navigate_to_pose");
}

void
NavigateToWaypoint::halt()
{
    // Si la navegación está activa, cancelar
    std::lock_guard<std::mutex> lock(mutex_);
    if (navigation_active_) {
        RCLCPP_INFO(node_->get_logger(), "Cancelando navegación");
        // La cancelación real se manejaría aquí si tuviéramos acceso al goal_handle
    }
}

BT::NodeStatus
NavigateToWaypoint::tick()
{
    // Comprobar si ya hemos terminado
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (navigation_succeeded_) {
            navigation_succeeded_ = false;
            navigation_active_ = false;
            return BT::NodeStatus::SUCCESS;
        }
        if (navigation_aborted_) {
            navigation_aborted_ = false;
            navigation_active_ = false;
            return BT::NodeStatus::FAILURE;
        }
    }

    // Si la navegación está en curso, esperar
    if (navigation_active_) {
        return BT::NodeStatus::RUNNING;
    }

    // Obtener el waypoint
    geometry_msgs::msg::PoseStamped waypoint;
    if (!getInput("waypoint", waypoint)) {
        RCLCPP_ERROR(node_->get_logger(), "No se ha proporcionado un waypoint");
        return BT::NodeStatus::FAILURE;
    }

    // Esperar a que el servidor de navegación esté disponible
    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Servidor de navegación no disponible");
        return BT::NodeStatus::FAILURE;
    }

    // Crear objetivo de navegación
    auto goal_msg = NavigateAction::Goal();
    goal_msg.pose = waypoint;

    RCLCPP_INFO(node_->get_logger(), 
              "Navegando a waypoint: x=%f, y=%f", 
              waypoint.pose.position.x, waypoint.pose.position.y);

    // Configurar opciones para envío del objetivo
    auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigateToWaypoint::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigateToWaypoint::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NavigateToWaypoint::result_callback, this, std::placeholders::_1);

    // Enviar el objetivo
    navigation_active_ = true;
    nav_action_client_->async_send_goal(goal_msg, send_goal_options);

    return BT::NodeStatus::RUNNING;
}

void
NavigateToWaypoint::goal_response_callback(NavigateGoalHandle::SharedPtr goal_handle)
{
    if (!goal_handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_ERROR(node_->get_logger(), "Objetivo rechazado por el servidor");
        navigation_active_ = false;
        navigation_aborted_ = true;
    } else {
        RCLCPP_INFO(node_->get_logger(), "Objetivo aceptado por el servidor");
    }
}

void
NavigateToWaypoint::feedback_callback(
    NavigateGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateAction::Feedback> feedback)
{
    (void)goal_handle;  // Evitar advertencia de variable no utilizada
    
    // Mostrar posición actual
    double distance_remaining = feedback->distance_remaining;
    RCLCPP_INFO(node_->get_logger(), "Distancia restante: %f", distance_remaining);
}

void
NavigateToWaypoint::result_callback(const NavigateGoalHandle::WrappedResult & result)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Procesar resultado
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "Navegación completada con éxito");
            navigation_succeeded_ = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "Navegación abortada");
            navigation_aborted_ = true;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(), "Navegación cancelada");
            navigation_aborted_ = true;
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Resultado de navegación desconocido");
            navigation_aborted_ = true;
            break;
    }
}

}  // namespace kobuki_yolo_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<kobuki_yolo_bt::NavigateToWaypoint>("NavigateToWaypoint");
} 