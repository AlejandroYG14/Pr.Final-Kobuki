#include "kobuki_yolo_bt/BehaviorTreeNode.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace kobuki_yolo_bt
{

using std::placeholders::_1;

BehaviorTreeNode::BehaviorTreeNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("behavior_tree_node");
    object_request_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/object_request", 10,
        std::bind(&BehaviorTreeNode::object_request_callback, this, _1));
}

BT::NodeStatus BehaviorTreeNode::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "Behavior Tree Node started");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BehaviorTreeNode::onRunning()
{
    if (!object_request_received_)
    {
        return BT::NodeStatus::RUNNING;
    }

    // Aquí se implementaría la lógica del Behavior Tree
    // Por ahora solo simulamos el comportamiento
    RCLCPP_INFO(node_->get_logger(), "Processing object request: %s", 
                requested_object_.c_str());
    
    return BT::NodeStatus::SUCCESS;
}

void BehaviorTreeNode::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "Behavior Tree Node halted");
}

void BehaviorTreeNode::object_request_callback(const std_msgs::msg::String::SharedPtr msg)
{
    requested_object_ = msg->data;
    object_request_received_ = true;
    RCLCPP_INFO(node_->get_logger(), "Received object request: %s", 
                requested_object_.c_str());
}

// Registro de los nodos del Behavior Tree
static void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<kobuki_yolo_bt::BehaviorTreeNode>("BehaviorTreeNode");
}

// Función para crear el Behavior Tree
BT::Tree createBehaviorTree()
{
    BT::BehaviorTreeFactory factory;
    RegisterNodes(factory);
    
    // Cargar el árbol desde el archivo XML
    return factory.createTreeFromFile("config/kobuki_fetch_object_bt.xml");
}

} // namespace kobuki_yolo_bt 