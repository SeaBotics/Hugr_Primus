#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hugr_hardware/serial_thruster_interface.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace hugr_hardware {

hardware_interface::CallbackReturn SerialThrusterInterface::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  auto & p = info_.hardware_parameters;
  device_ = p.at("device");
  baudrate_ = std::stoi(p.at("baudrate"));
  min_thrust_N_ = std::stod(p.at("min_thrust_N"));
  max_thrust_N_ = std::stod(p.at("max_thrust_N"));
  if (p.find("deadband_N") != p.end()) deadband_N_ = std::stod(p.at("deadband_N"));

  // Bruk joints fra URDF som thruster-liste
  for (auto & j : info_.joints) {
    Thruster t; t.name = j.name; t.joint = j.name; t.axis[0]=1; t.axis[1]=0; t.axis[2]=0;
    thrusters_.push_back(t);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SerialThrusterInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> si;
  for (auto & t : thrusters_)
    si.emplace_back(hardware_interface::StateInterface(t.joint, hardware_interface::HW_IF_EFFORT, &t.eff_N));
  return si;
}

std::vector<hardware_interface::CommandInterface> SerialThrusterInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> ci;
  for (auto & t : thrusters_)
    ci.emplace_back(hardware_interface::CommandInterface(t.joint, hardware_interface::HW_IF_EFFORT, &t.cmd_N));
  return ci;
}

hardware_interface::CallbackReturn SerialThrusterInterface::on_activate(const rclcpp_lifecycle::State &) {
  // Minimal serial open
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("SerialThrusterInterface"), "Failed to open %s", device_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  termios tio{}; tcgetattr(fd_, &tio); cfmakeraw(&tio);
  cfsetispeed(&tio, B115200); cfsetospeed(&tio, B115200);
  tio.c_cflag |= (CLOCAL | CREAD);
  tcsetattr(fd_, TCSANOW, &tio);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SerialThrusterInterface::on_deactivate(const rclcpp_lifecycle::State &) {
  closeSerial(); return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SerialThrusterInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
  // Ingen feedback ennå — speil cmd som state
  for (auto & t : thrusters_) t.eff_N = t.cmd_N;
  return hardware_interface::return_type::OK;
}

static uint16_t clip_map(double v, double vmin, double vmax) {
  if (v < vmin) v = vmin; if (v > vmax) v = vmax;
  double span = vmax - vmin;
  double norm = (v - vmin) / span; // 0..1
  double pwm = 1100.0 + norm * 800.0; // [1100..1900]
  if (pwm < 1100) pwm = 1100; if (pwm > 1900) pwm = 1900;
  return static_cast<uint16_t>(pwm);
}

hardware_interface::return_type SerialThrusterInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
  if (fd_ < 0) return hardware_interface::return_type::ERROR;
  uint8_t buf[2 + 4*2 + 1]; buf[0]=0xAA; buf[1]=0x55; size_t idx=2;
  for (auto & t : thrusters_) {
    uint16_t pwm = clip_map(t.cmd_N, min_thrust_N_, max_thrust_N_);
    buf[idx++] = pwm & 0xFF; buf[idx++] = (pwm >> 8) & 0xFF;
  }
  uint8_t crc=0; for (size_t i=0;i<idx;i++) crc ^= buf[i]; buf[idx]=crc; idx++;
  ::write(fd_, buf, idx);
  return hardware_interface::return_type::OK;
}

void SerialThrusterInterface::closeSerial() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

} // namespace hugr_hardware