#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace autopilot
{

class AutopilotNode : public rclcpp::Node
{
public:
  explicit AutopilotNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParameters();
  void loadParameters();
  void controlLoop();

  // Publisher: enkel placeholder for thruster commands
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub_;

  // Timer for kontrollsløyfe
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Parametere
  double control_rate_hz_;
  double test_port_surge_;
  double test_starboard_surge_;
  double test_port_tunnel_;
  double test_starboard_tunnel_;

  // Fremtidig state
  bool system_enabled_;
};

}  // namespace asv_autopilot