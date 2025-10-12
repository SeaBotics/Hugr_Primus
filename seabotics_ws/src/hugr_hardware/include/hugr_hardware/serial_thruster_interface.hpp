#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace hugr_hardware
{

class SerialThrusterInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SerialThrusterInterface)

  // ROS 2 Humble: on_init / on_activate / on_deactivate
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
      const rclcpp::Time &, const rclcpp::Duration &) override;

  hardware_interface::return_type write(
      const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // logger (vi er ikke en Node)
  rclcpp::Logger logger_ = rclcpp::get_logger("serial_thruster_interface");

  // seriell info
  std::string port_{"/dev/ttyUSB0"};
  int baudrate_{115200};
  int fd_{-1};

  // state/command buffers (én per joint)
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

} // namespace hugr_hardware
