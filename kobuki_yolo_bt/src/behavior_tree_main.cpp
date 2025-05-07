#include "kobuki_yolo_bt/BehaviorTreeNode.hpp"
#include "kobuki_yolo_bt/BehaviorTreeFactory.hpp"
#include "kobuki_yolo_bt/GetWaypoint.hpp"
#include "kobuki_yolo_bt/NavigateToWaypoint.hpp"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("behavior_tree_node");
    
    // Crear blackboard
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    
    // Compartir el nodo en el blackboard para ser usado por otros nodos
    blackboard->set("node", node);
    
    // Inicializar fábrica de BT
    BT::BehaviorTreeFactory factory;
    
    // Registrar nodos del BT
    factory.registerNodeType<kobuki_yolo_bt::BehaviorTreeNode>("BehaviorTreeNode");
    factory.registerNodeType<kobuki_yolo_bt::GetWaypoint>("GetWaypoint");
    factory.registerNodeType<kobuki_yolo_bt::NavigateToWaypoint>("NavigateToWaypoint");
    
    // Crear árbol a partir del archivo XML
    auto tree = factory.createTreeFromFile("config/kobuki_fetch_object_bt.xml", blackboard);
    
    // Bucle principal
    RCLCPP_INFO(node->get_logger(), "Iniciando Behavior Tree");
    while (rclcpp::ok())
    {
        // Ejecutar un tick del árbol
        tree.tickRoot();
        
        // Procesar callbacks de ROS
        rclcpp::spin_some(node);
        
        // Pequeña pausa para no saturar la CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    rclcpp::shutdown();
    return 0;
} 