#ifndef KOBUKI_YOLO_BT__NAVIGATE_TO_WAYPOINT_HPP_
#define KOBUKI_YOLO_BT__NAVIGATE_TO_WAYPOINT_HPP_

#include <string>
#include <memory>

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace kobuki_yolo_bt
{

class NavigateToWaypoint : public BT::ActionNodeBase
{
public:
    NavigateToWaypoint(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("waypoint")
        };
    }

    void halt() override;
    BT::NodeStatus tick() override;

private:
    // Definiciones del cliente de acción
    using NavigateAction = nav2_msgs::action::NavigateToPose;
    using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateAction>;

    // Callback para respuesta del objetivo
    void goal_response_callback(NavigateGoalHandle::SharedPtr goal_handle);

    // Callback para realimentación
    void feedback_callback(
        NavigateGoalHandle::SharedPtr goal_handle,
        const std::shared_ptr<const NavigateAction::Feedback> feedback);

    // Callback para resultado
    void result_callback(const NavigateGoalHandle::WrappedResult & result);

    // Cliente de acción para navegación
    rclcpp_action::Client<NavigateAction>::SharedPtr nav_action_client_;
    
    // Nodo ROS
    rclcpp::Node::SharedPtr node_;
    
    // Estado de navegación
    bool navigation_succeeded_;
    bool navigation_active_;
    bool navigation_aborted_;
    
    // Mutex para sincronización
    std::mutex mutex_;
};

}  // namespace kobuki_yolo_bt

#endif  // KOBUKI_YOLO_BT__NAVIGATE_TO_WAYPOINT_HPP_ 