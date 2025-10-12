#include "hugr_hardware/serial_thruster_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <cerrno>
#include <cstring>
#include <fcntl.h>      // open()
#include <termios.h>    // serial settings
#include <unistd.h>     // read, write

namespace hugr_hardware
{

// -----------------------------------------------------------
//  INIT
// -----------------------------------------------------------
hardware_interface::CallbackReturn
SerialThrusterInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info)
      != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("port")) {
    port_ = info_.hardware_parameters.at("port");
  }
  if (info_.hardware_parameters.count("baudrate")) {
    baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
  }

  RCLCPP_INFO(logger_, "Opening serial port %s @ %d baud", port_.c_str(), baudrate_);

  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(logger_, "open(%s) failed: %s", port_.c_str(), std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcgetattr failed: %s", std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 8N1 @ 115200 (bruk B115200 konstant)
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_lflag = 0;
  tty.c_iflag = 0;
  tty.c_oflag = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcsetattr failed: %s", std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.assign(info_.joints.size(), 0.0);
  hw_states_.assign(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------
//  ACTIVATE / DEACTIVATE (HUMBLE SIGNATURES)
// -----------------------------------------------------------
hardware_interface::CallbackReturn
SerialThrusterInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "SerialThrusterInterface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
SerialThrusterInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "SerialThrusterInterface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------
//  EXPORTS
// -----------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SerialThrusterInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  states.reserve(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    states.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION,
                        &hw_states_[i]);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface>
SerialThrusterInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;
  cmds.reserve(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    cmds.emplace_back(info_.joints[i].name,
                      hardware_interface::HW_IF_POSITION,
                      &hw_commands_[i]);
  }
  return cmds;
}

// -----------------------------------------------------------
//  WRITE
// -----------------------------------------------------------
hardware_interface::return_type
SerialThrusterInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Pakk veldig enkelt: "T0:val;T1:val;...;\n"
  uint8_t buf[128];
  int idx = 0;
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    idx += std::snprintf(reinterpret_cast<char*>(buf) + idx,
                         sizeof(buf) - idx, "T%zu:%.2f;", i, hw_commands_[i]);
    if (idx >= static_cast<int>(sizeof(buf) - 8)) break;
  }
  buf[idx++] = '\n';

  size_t to_write = static_cast<size_t>(idx);
  const uint8_t* p = buf;

  while (to_write > 0) {
    ssize_t n = ::write(fd_, p, to_write);
    if (n < 0) {
      RCLCPP_ERROR(logger_, "write failed: %s", std::strerror(errno));
      return hardware_interface::return_type::ERROR;
    }
    if (n == 0) {
      RCLCPP_WARN(logger_, "write returned 0, retrying...");
      continue;
    }
    p += static_cast<size_t>(n);
    to_write -= static_cast<size_t>(n);
  }

  if (to_write != 0) {
    RCLCPP_WARN(logger_, "Partial write: %zu bytes left (requested %d)",
                to_write, idx);
  }

  return hardware_interface::return_type::OK;
}

// -----------------------------------------------------------
//  READ
// -----------------------------------------------------------
hardware_interface::return_type
SerialThrusterInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  hw_states_ = hw_commands_; // dummy feedback
  return hardware_interface::return_type::OK;
}

} // namespace hugr_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hugr_hardware::SerialThrusterInterface,
                       hardware_interface::SystemInterface)
