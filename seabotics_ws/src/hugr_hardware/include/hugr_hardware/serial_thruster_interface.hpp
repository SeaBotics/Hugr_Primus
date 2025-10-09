#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

namespace hugr_hardware {

class SerialThrusterInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SerialThrusterInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string device_;
  int baudrate_ = 115200;
  double min_thrust_N_ = -55.0, max_thrust_N_ = 55.0, deadband_N_ = 2.0;

  struct Thruster {
    std::string name;
    std::string joint;
    double axis[3];
    double cmd_N = 0.0;
    double eff_N = 0.0;
  };
  std::vector<Thruster> thrusters_;

  int fd_ = -1;
  bool openSerial();
  void closeSerial();
};

} // namespace hugr_hardware
