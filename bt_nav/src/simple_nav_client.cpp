// Copyright 2021 Intelligent Robotics Lab
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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class SimpleNavigationClient : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  SimpleNavigationClient()
  : Node("simple_navigation_client")
  {
    this->client_ptr_ = rclcpp_action::create_client<FollowWaypoints>(
      this,
      "follow_waypoints");

    RCLCPP_INFO(this->get_logger(), "Navigation client initialized. Waiting for action server...");
  }

  void navigate_waypoints()
  {
    // Esperar a que el servidor de acción esté disponible
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Action server available, sending navigation goal");

    // Crear lista de waypoints
    auto waypoints = std::make_shared<FollowWaypoints::Goal>();

    // Crear waypoint principal (2.66, -9.12)
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header.frame_id = "map";
    waypoint.header.stamp = this->now();
    waypoint.pose.position.x = 2.66;
    waypoint.pose.position.y = -9.12;
    waypoint.pose.position.z = 0.0;
    waypoint.pose.orientation.w = 1.0;

    // Crear waypoint origen (0.0, 0.0)
    geometry_msgs::msg::PoseStamped origin;
    origin.header.frame_id = "map";
    origin.header.stamp = this->now();
    origin.pose.position.x = 0.0;
    origin.pose.position.y = 0.0;
    origin.pose.position.z = 0.0;
    origin.pose.orientation.w = 1.0;

    // Añadir waypoints a la lista
    waypoints->poses.push_back(waypoint);
    waypoints->poses.push_back(origin);

    // Enviar goal
    RCLCPP_INFO(this->get_logger(), "Sending goal to navigate to point (2.66, -9.12) and then return to origin");

    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      [this](const GoalHandleFollowWaypoints::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
      };

    send_goal_options.feedback_callback =
      [this](
      GoalHandleFollowWaypoints::SharedPtr,
      const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
        RCLCPP_INFO(
          this->get_logger(), "Current waypoint: %d", 
          feedback->current_waypoint);
      };

    send_goal_options.result_callback =
      [this](const GoalHandleFollowWaypoints::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation completed successfully!");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
      };

    this->client_ptr_->async_send_goal(waypoints, send_goal_options);
  }

private:
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_ptr_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto navigation_client = std::make_shared<SimpleNavigationClient>();
  
  // Dar tiempo para que los publishers y subscribers se inicialicen
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  // Iniciar navegación
  navigation_client->navigate_waypoints();
  
  rclcpp::spin(navigation_client);
  rclcpp::shutdown();
  
  return 0;
} 