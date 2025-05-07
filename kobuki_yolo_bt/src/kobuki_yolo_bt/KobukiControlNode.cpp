#include "kobuki_yolo_bt/KobukiControlNode.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

namespace kobuki_yolo_bt
{

using std::placeholders::_1;

KobukiControlNode::KobukiControlNode()
: Node("kobuki_control_node")
{
    // Suscripciones
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_command", 10,
        std::bind(&KobukiControlNode::command_callback, this, _1));
        
    // Publicadores
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_status", 10);
    
    RCLCPP_INFO(this->get_logger(), "Kobuki Control Node initialized");
}

void KobukiControlNode::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received command: %s", command.c_str());
    
    if (command == "stop")
    {
        stop_robot();
    }
    else if (command == "move_forward")
    {
        move_forward();
    }
    else if (command == "move_backward")
    {
        move_backward();
    }
    else if (command == "turn_left")
    {
        turn_left();
    }
    else if (command == "turn_right")
    {
        turn_right();
    }
    else if (command == "pick_object")
    {
        pick_object();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
    }
}

void KobukiControlNode::stop_robot()
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
    
    std_msgs::msg::String status_msg;
    status_msg.data = "stopped";
    status_pub_->publish(status_msg);
}

void KobukiControlNode::move_forward()
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.2;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
    
    std_msgs::msg::String status_msg;
    status_msg.data = "moving_forward";
    status_pub_->publish(status_msg);
}

void KobukiControlNode::move_backward()
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = -0.2;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
    
    std_msgs::msg::String status_msg;
    status_msg.data = "moving_backward";
    status_pub_->publish(status_msg);
}

void KobukiControlNode::turn_left()
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.5;
    cmd_vel_pub_->publish(cmd_vel);
    
    std_msgs::msg::String status_msg;
    status_msg.data = "turning_left";
    status_pub_->publish(status_msg);
}

void KobukiControlNode::turn_right()
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = -0.5;
    cmd_vel_pub_->publish(cmd_vel);
    
    std_msgs::msg::String status_msg;
    status_msg.data = "turning_right";
    status_pub_->publish(status_msg);
}

void KobukiControlNode::pick_object()
{
    // Simular la recogida del objeto
    RCLCPP_INFO(this->get_logger(), "Picking object...");
    
    std_msgs::msg::String status_msg;
    status_msg.data = "object_picked";
    status_pub_->publish(status_msg);
}

} // namespace kobuki_yolo_bt 