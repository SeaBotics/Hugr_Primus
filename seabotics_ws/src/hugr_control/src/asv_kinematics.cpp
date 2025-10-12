#include <rclcpp/rclcpp.hpp>

class ASVKinematics : public rclcpp::Node {
public:
  ASVKinematics() : rclcpp::Node("asv_kinematics") {
    this->declare_parameter("mass", 50.0);
    this->declare_parameter("surge_gain", 0.08);
    this->declare_parameter("sway_gain", 0.08);
    this->declare_parameter("yaw_gain", 0.25);
    this->declare_parameter("thruster_max", 50.0);
    RCLCPP_INFO(this->get_logger(), "asv_kinematics started (stub).");
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ASVKinematics>());
  rclcpp::shutdown();
  return 0;
}
