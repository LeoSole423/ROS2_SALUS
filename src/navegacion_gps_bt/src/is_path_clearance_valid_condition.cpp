#include "navegacion_gps_bt/is_path_clearance_valid_condition.hpp"

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace navegacion_gps_bt
{

IsPathClearanceValidCondition::IsPathClearanceValidCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::IsPathValid>(
    condition_name,
    conf,
    "/path_clearance_validator/is_path_clearance_valid")
{
}

void IsPathClearanceValidCondition::on_tick()
{
  nav_msgs::msg::Path path;
  if (!getInput<nav_msgs::msg::Path>("path", path)) {
    RCLCPP_WARN(node_->get_logger(), "IsPathClearanceValid missing required input [path]");
    should_send_request_ = false;
    return;
  }
  request_->path = path;
}

BT::NodeStatus IsPathClearanceValidCondition::on_completion(
  std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
{
  return response->is_valid ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace navegacion_gps_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navegacion_gps_bt::IsPathClearanceValidCondition>(
    "IsPathClearanceValid");
}
