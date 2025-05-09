#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Incluir la clase Seeker
#include "bt_nav/TFSeekerNode.hpp"

// Función para ejecutar el comportamiento Seeker
void executeSeekerBehavior(rclcpp::Node::SharedPtr node) {
  RCLCPP_INFO(node->get_logger(), "Executing Seeker behavior");
  
  // Crear instancia del nodo Seeker
  auto seeker_node = std::make_shared<tf_seeker::TFSeekerNode>();
  
  // Tiempo máximo de búsqueda (20 segundos)
  const auto max_search_time = std::chrono::seconds(20);
  auto start_time = node->now();
  
  RCLCPP_INFO(node->get_logger(), "Seeker: Buscando persona...");
  
  while (rclcpp::ok()) {
    // Procesar el nodo ROS
    rclcpp::spin_some(seeker_node);
    
    // Si ya alcanzó la meta, salimos
    if (seeker_node->has_reached_goal()) {
      RCLCPP_INFO(node->get_logger(), "Seeker: Persona alcanzada.");
      break;
    }
    
    // Verificar si se ha agotado el tiempo de búsqueda
    auto elapsed = node->now() - start_time;
    if (elapsed > rclcpp::Duration::from_seconds(max_search_time.count())) {
      RCLCPP_WARN(node->get_logger(), "Seeker: Tiempo de búsqueda agotado.");
      // Enviar comando de parada
      geometry_msgs::msg::Twist stop_cmd;
      seeker_node->get_vel_publisher()->publish(stop_cmd);
      break;
    }
    
    // Breve pausa para no saturar CPU
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  RCLCPP_INFO(node->get_logger(), "Seeker: Task completed");
}

// ... (includes y funciones sin cambios)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("nav_waypoints_client");

  auto action_client = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    node, "follow_waypoints");

  if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "Action server 'follow_waypoints' not available after waiting");
    return 1;
  }

  geometry_msgs::msg::PoseStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = node->now();
  waypoint.pose.position.x = 4.080453395843506;
  waypoint.pose.position.y = -7.847793102264404;
  waypoint.pose.position.z = -0.001434326171875;
  waypoint.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped origin;
  origin.header.frame_id = "map";
  origin.header.stamp = node->now();
  origin.pose.position.x = 0;
  origin.pose.position.y = 0;
  origin.pose.position.z = 0.0;
  origin.pose.orientation.w = 1.0;

  bool step_completed = false;
  bool failure_occurred = false;

  // Paso 1: ir al waypoint
  {
    RCLCPP_INFO(node->get_logger(), "Step 1: Moving to waypoint");

    auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
    goal_msg.poses.push_back(waypoint);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();

    send_goal_options.result_callback =
      [node, &step_completed, &failure_occurred](auto result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(node->get_logger(), "Navigation to waypoint succeeded");
          step_completed = true;
        } else {
          RCLCPP_ERROR(node->get_logger(), "Navigation to waypoint failed");
          failure_occurred = true;
        }
      };

    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

    while (rclcpp::ok() && !step_completed && !failure_occurred) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // Paso 2: Seeker
  if (!failure_occurred) {
    try {
      RCLCPP_INFO(node->get_logger(), "Step 2: Executing Seeker behavior");
      executeSeekerBehavior(node);
    } catch (...) {
      RCLCPP_ERROR(node->get_logger(), "Seeker behavior failed");
      failure_occurred = true;
    }
  }

  // Paso 3: Volver al origen SI hubo fallo
  if (failure_occurred || step_completed) {
    RCLCPP_INFO(node->get_logger(), "Step 3: Returning to origin");

    auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
    goal_msg.poses.push_back(origin);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();

    send_goal_options.result_callback =
      [node](auto result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(node->get_logger(), "Return to origin succeeded");
        } else {
          RCLCPP_ERROR(node->get_logger(), "Return to origin failed");
        }
        rclcpp::shutdown();
      };

    action_client->async_send_goal(goal_msg, send_goal_options);
    rclcpp::spin(node);
  } else {
    rclcpp::shutdown();
  }

  return 0;
}
