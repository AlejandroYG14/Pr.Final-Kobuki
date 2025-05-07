#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/bool.hpp"

class StationNavigatorNode : public rclcpp::Node
{
public:
  StationNavigatorNode() : Node("station_navigator"), current_station_(0)
  {
    // Crear publisher para goal pose
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10);

    // Crear timer para publicar los waypoints
    timer_ = create_wall_timer(
      std::chrono::seconds(30),
      std::bind(&StationNavigatorNode::publish_next_waypoint, this));

    // Inicializar waypoints
    init_waypoints();
  }

private:
  void init_waypoints()
  {
    // Estación al lado de la puerta
    geometry_msgs::msg::PoseStamped station1;
    station1.header.frame_id = "map";
    station1.pose.position.x = 3.7830708026885986;
    station1.pose.position.y = -9.048851013183594;
    station1.pose.position.z = -0.001373291015625;
    station1.pose.orientation.w = 1.0;
    waypoints_.push_back(station1);

    // Estación al lado de la mesa grande
    geometry_msgs::msg::PoseStamped station2;
    station2.header.frame_id = "map";
    station2.pose.position.x = 4.615912914276123;
    station2.pose.position.y = -4.381537914276123;
    station2.pose.position.z = -0.0013427734375;
    station2.pose.orientation.w = 1.0;
    waypoints_.push_back(station2);
  }

  void publish_next_waypoint()
  {
    if (waypoints_.empty()) {
      return;
    }

    // Publicar el waypoint actual
    auto waypoint = waypoints_[current_station_];
    waypoint.header.stamp = now();
    goal_publisher_->publish(waypoint);

    RCLCPP_INFO(get_logger(), "Navegando a estación %zu: x=%.2f, y=%.2f",
                current_station_ + 1, waypoint.pose.position.x, waypoint.pose.position.y);

    // Avanzar al siguiente waypoint de forma circular
    current_station_ = (current_station_ + 1) % waypoints_.size();
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  size_t current_station_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StationNavigatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}