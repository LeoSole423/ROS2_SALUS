#ifndef NAVEGACION_GPS_BT__IS_PATH_CLEARANCE_VALID_CONDITION_HPP_
#define NAVEGACION_GPS_BT__IS_PATH_CLEARANCE_VALID_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace navegacion_gps_bt
{

class IsPathClearanceValidCondition
  : public nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::IsPathValid>
{
public:
  IsPathClearanceValidCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to check for clearance"),
      });
  }

  void on_tick() override;

  BT::NodeStatus on_completion(
    std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response) override;
};

}  // namespace navegacion_gps_bt

#endif  // NAVEGACION_GPS_BT__IS_PATH_CLEARANCE_VALID_CONDITION_HPP_
