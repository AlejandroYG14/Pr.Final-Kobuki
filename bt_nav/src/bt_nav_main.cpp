#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Function to execute the Seeker behavior
void executeSeekerBehavior(rclcpp::Node::SharedPtr node) {
  RCLCPP_INFO(node->get_logger(), "Executing Seeker behavior");
  
  // Simulate seeker behavior for demonstration
  RCLCPP_INFO(node->get_logger(), "Seeker: Scanning surroundings...");
  std::this_thread::sleep_for(std::chrono::seconds(3));
  RCLCPP_INFO(node->get_logger(), "Seeker: Task completed");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("nav_waypoints_client");

  RCLCPP_INFO(node->get_logger(), "Creating follow_waypoints action client");
  auto action_client = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    node, "follow_waypoints");

  // Esperar a que el servidor de acción esté disponible
  if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "Action server 'follow_waypoints' not available after waiting");
    return 1;
  }

  // Crear waypoint principal con las nuevas coordenadas
  geometry_msgs::msg::PoseStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = node->now();
  waypoint.pose.position.x = 4.080453395843506;
  waypoint.pose.position.y = -7.847793102264404;
  waypoint.pose.position.z = -0.001434326171875;
  waypoint.pose.orientation.w = 1.0;

  // Crear waypoint origen (0.0, 0.0)
  geometry_msgs::msg::PoseStamped origin;
  origin.header.frame_id = "map";
  origin.header.stamp = node->now();
  origin.pose.position.x = 0.0;
  origin.pose.position.y = 0.0;
  origin.pose.position.z = 0.0;
  origin.pose.orientation.w = 1.0;

  // Primero, vamos al waypoint principal
  {
    RCLCPP_INFO(node->get_logger(), "Step 1: Moving to waypoint (%.3f, %.3f, %.3f)",
                waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
    
    auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
    goal_msg.poses.push_back(waypoint);
    
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      [node](auto goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(node->get_logger(), "Goal to waypoint was rejected by server");
        } else {
          RCLCPP_INFO(node->get_logger(), "Goal to waypoint accepted by server, navigation started");
        }
      };

    send_goal_options.feedback_callback =
      [node](auto, auto feedback) {
        RCLCPP_INFO(
          node->get_logger(), "Currently executing waypoint %d", 
          feedback->current_waypoint);
      };

    bool step_completed = false;
    send_goal_options.result_callback =
      [node, &step_completed](auto result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node->get_logger(), "Navigation to waypoint completed successfully!");
            step_completed = true;
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node->get_logger(), "Goal to waypoint was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node->get_logger(), "Goal to waypoint was canceled");
            break;
          default:
            RCLCPP_ERROR(node->get_logger(), "Unknown result code");
            break;
        }
      };

    // Enviar goal
    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
    
    // Esperar a que se complete esta etapa
    while (rclcpp::ok() && !step_completed) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // Segundo, ejecutar comportamiento Seeker
  RCLCPP_INFO(node->get_logger(), "Step 2: Executing Seeker behavior");
  executeSeekerBehavior(node);

  // Tercero, volver al origen
  {
    RCLCPP_INFO(node->get_logger(), "Step 3: Returning to origin (0, 0, 0)");
    
    auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
    goal_msg.poses.push_back(origin);
    
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      [node](auto goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(node->get_logger(), "Goal to origin was rejected by server");
        } else {
          RCLCPP_INFO(node->get_logger(), "Goal to origin accepted by server, navigation started");
        }
      };

    send_goal_options.feedback_callback =
      [node](auto, auto feedback) {
        RCLCPP_INFO(
          node->get_logger(), "Currently executing waypoint %d", 
          feedback->current_waypoint);
      };

    send_goal_options.result_callback =
      [node](auto result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node->get_logger(), "Navigation back to origin completed successfully!");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node->get_logger(), "Goal to origin was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node->get_logger(), "Goal to origin was canceled");
            break;
          default:
            RCLCPP_ERROR(node->get_logger(), "Unknown result code");
            break;
        }
        rclcpp::shutdown();
      };

    // Enviar goal
    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
  }
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
