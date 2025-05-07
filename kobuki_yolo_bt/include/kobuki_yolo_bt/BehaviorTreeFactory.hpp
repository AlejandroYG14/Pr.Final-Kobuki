#ifndef KOBUKI_YOLO_BT__BEHAVIOR_TREE_FACTORY_HPP_
#define KOBUKI_YOLO_BT__BEHAVIOR_TREE_FACTORY_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include "kobuki_yolo_bt/BehaviorTreeNode.hpp"

namespace kobuki_yolo_bt
{

// Función para crear el Behavior Tree
inline BT::Tree createBehaviorTree()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<kobuki_yolo_bt::BehaviorTreeNode>("BehaviorTreeNode");
    
    // Cargar el árbol desde el archivo XML
    return factory.createTreeFromFile("config/kobuki_fetch_object_bt.xml");
}

} // namespace kobuki_yolo_bt

#endif // KOBUKI_YOLO_BT__BEHAVIOR_TREE_FACTORY_HPP_ 