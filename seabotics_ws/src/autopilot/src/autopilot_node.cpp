#include "autopilot/autopilot_node.hpp"

#include <vector>
#include <chrono>
#include <utility>

using namespace std::chrono_literals;

namespace autopilot
{

AutopilotNode::AutopilotNode(const rclcpp::NodeOptions & options)
: Node("autopilot_node", options),
  control_rate_hz_(20.0),
  test_port_surge_(0.0),
  test_starboard_surge_(0.0),
  test_port_tunnel_(0.0),
  test_starboard_tunnel_(0.0),
  system_enabled_(true)
{
  declareParameters();
  loadParameters();

  thruster_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/control/thruster_commands", 10);

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);

  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&AutopilotNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Autopilot node started");
  RCLCPP_INFO(
    this->get_logger(),
    "Control rate: %.2f Hz",
    control_rate_hz_);
}

void AutopilotNode::declareParameters()
{
  this->declare_parameter<double>("control_rate_hz", 20.0);

  // Testverdier for første bring-up
  this->declare_parameter<double>("test_port_surge", 0.0);
  this->declare_parameter<double>("test_starboard_surge", 0.0);
  this->declare_parameter<double>("test_port_tunnel", 0.0);
  this->declare_parameter<double>("test_starboard_tunnel", 0.0);

  this->declare_parameter<bool>("system_enabled", true);
}

void AutopilotNode::loadParameters()
{
  control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();

  test_port_surge_ = this->get_parameter("test_port_surge").as_double();
  test_starboard_surge_ = this->get_parameter("test_starboard_surge").as_double();
  test_port_tunnel_ = this->get_parameter("test_port_tunnel").as_double();
  test_starboard_tunnel_ = this->get_parameter("test_starboard_tunnel").as_double();

  system_enabled_ = this->get_parameter("system_enabled").as_bool();
}

void AutopilotNode::controlLoop()
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(4, 0.0);

  if (!system_enabled_) {
    // Alt nulles hvis node er deaktivert
    msg.data[0] = 0.0;  // port_surge
    msg.data[1] = 0.0;  // starboard_surge
    msg.data[2] = 0.0;  // port_tunnel
    msg.data[3] = 0.0;  // starboard_tunnel
  } else {
    // Foreløpig: bare publiser testverdier
    msg.data[0] = test_port_surge_;
    msg.data[1] = test_starboard_surge_;
    msg.data[2] = test_port_tunnel_;
    msg.data[3] = test_starboard_tunnel_;
  }

  thruster_pub_->publish(msg);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Published thrusters: [%.3f, %.3f, %.3f, %.3f]",
    msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
}

}  // namespace asv_autopilot