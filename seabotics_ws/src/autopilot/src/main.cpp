#include "rclcpp/rclcpp.hpp"
#include "autopilot/autopilot_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<autopilot::AutopilotNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}