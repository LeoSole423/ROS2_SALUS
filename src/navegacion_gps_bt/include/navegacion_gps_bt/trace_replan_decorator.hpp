#ifndef NAVEGACION_GPS_BT__TRACE_REPLAN_DECORATOR_HPP_
#define NAVEGACION_GPS_BT__TRACE_REPLAN_DECORATOR_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interfaces/msg/nav_event.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace navegacion_gps_bt
{

class TraceReplanDecorator : public BT::DecoratorNode
{
public:
  TraceReplanDecorator(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;

private:
  void publish_event(const std::string & code, const std::string & outcome);
  static diagnostic_msgs::msg::KeyValue detail(const std::string & key, const std::string & value);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<interfaces::msg::NavEvent>::SharedPtr publisher_;
  std::chrono::steady_clock::time_point started_at_;
  std::string reason_;
  uint32_t replan_id_{0};
  size_t goals_before_{0};
  bool active_{false};
};

}  // namespace navegacion_gps_bt

#endif  // NAVEGACION_GPS_BT__TRACE_REPLAN_DECORATOR_HPP_
