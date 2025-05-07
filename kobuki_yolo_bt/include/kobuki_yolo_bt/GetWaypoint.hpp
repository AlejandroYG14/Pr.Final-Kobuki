#ifndef KOBUKI_YOLO_BT__GET_WAYPOINT_HPP_
#define KOBUKI_YOLO_BT__GET_WAYPOINT_HPP_

#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace kobuki_yolo_bt
{

class GetWaypoint : public BT::ActionNodeBase
{
public:
    GetWaypoint(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint"),
            BT::InputPort<std::string>("station_id")
        };
    }

    void halt() override;
    BT::NodeStatus tick() override;

private:
    geometry_msgs::msg::PoseStamped getStationPose(const std::string& station_id);

    std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> stations_;
    geometry_msgs::msg::PoseStamped wp_;
    size_t current_station_index_;
    std::chrono::time_point<std::chrono::steady_clock> last_station_change_;
    std::chrono::seconds timeout_{30}; // 30 segundos de timeout por defecto
};

}  // namespace kobuki_yolo_bt

#endif  // KOBUKI_YOLO_BT__GET_WAYPOINT_HPP_ 