#include "navegacion_gps_bt/trace_replan_decorator.hpp"

#include <atomic>
#include <iomanip>
#include <sstream>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace navegacion_gps_bt
{

namespace
{
std::atomic<uint32_t> g_event_sequence{0};
std::atomic<uint32_t> g_replan_sequence{0};
}  // namespace

TraceReplanDecorator::TraceReplanDecorator(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  publisher_ = node_->create_publisher<interfaces::msg::NavEvent>(
    "/navigation_trace/events", rclcpp::QoS(50));
}

BT::PortsList TraceReplanDecorator::providedPorts()
{
  return {
    BT::InputPort<std::string>("reason", "Cause that selected this replanning branch"),
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
    BT::InputPort<nav_msgs::msg::Path>("path")
  };
}

diagnostic_msgs::msg::KeyValue TraceReplanDecorator::detail(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

void TraceReplanDecorator::publish_event(
  const std::string & code, const std::string & outcome)
{
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  nav_msgs::msg::Path path;
  getInput("goals", goals);
  getInput("path", path);

  const auto elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started_at_).count();
  std::ostringstream duration_stream;
  duration_stream << std::fixed << std::setprecision(3) << (active_ ? elapsed : 0.0);

  interfaces::msg::NavEvent event;
  event.stamp = node_->now();
  event.severity = outcome == "failure" ?
    diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::OK;
  event.component = "bt_navigator";
  event.code = code;
  event.message = "NavigateThroughPoses replanning " + outcome;
  event.event_id = ++g_event_sequence;
  event.details = {
    detail("replan_id", std::to_string(replan_id_)),
    detail("reason", reason_),
    detail("outcome", outcome),
    detail("duration_ms", duration_stream.str()),
    detail("goals_before", std::to_string(goals_before_)),
    detail("goals_after", std::to_string(goals.size())),
    detail("path_pose_count", std::to_string(path.poses.size())),
    detail("path_frame", path.header.frame_id)
  };
  publisher_->publish(event);
}

BT::NodeStatus TraceReplanDecorator::tick()
{
  if (!active_) {
    reason_ = "unknown";
    getInput("reason", reason_);
    std::vector<geometry_msgs::msg::PoseStamped> goals;
    getInput("goals", goals);
    goals_before_ = goals.size();
    started_at_ = std::chrono::steady_clock::now();
    active_ = true;
    replan_id_ = ++g_replan_sequence;
    publish_event("REPLAN_STARTED", "started");
  }

  const BT::NodeStatus child_status = child_node_->executeTick();
  if (child_status == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }

  publish_event(
    child_status == BT::NodeStatus::SUCCESS ? "REPLAN_FINISHED" : "REPLAN_FAILED",
    child_status == BT::NodeStatus::SUCCESS ? "success" : "failure");
  active_ = false;
  resetChild();
  return child_status;
}

void TraceReplanDecorator::halt()
{
  if (active_) {
    publish_event("REPLAN_CANCELLED", "cancelled");
  }
  active_ = false;
  BT::DecoratorNode::halt();
}

}  // namespace navegacion_gps_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navegacion_gps_bt::TraceReplanDecorator>("TraceReplan");
}
