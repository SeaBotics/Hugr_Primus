#pragma once

#include <rviz_common/panel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32.hpp>

#include <QLabel>
#include <QProgressBar>
#include <QTimer>
#include <QString>

#include <mutex>
#include <thread>
#include <atomic>

namespace hugr_rviz_panel
{

class HugrStatusPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit HugrStatusPanel(QWidget * parent = nullptr);
  ~HugrStatusPanel() override;
  void onInitialize() override;

private Q_SLOTS:
  void onUiTimer();

private:
  void startRos();
  void stopRos();

  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void modeCb(const std_msgs::msg::Int8::SharedPtr msg);
  void batteryCb(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void runtimeCb(const std_msgs::msg::Float32::SharedPtr msg);
  void temperatureCb(const std_msgs::msg::Float32::SharedPtr msg);
  void odomLocalCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odomGlobalCb(const nav_msgs::msg::Odometry::SharedPtr msg);

  static void quatToRPY(double x, double y, double z, double w,
                        double & roll, double & pitch, double & yaw);

  QLabel * lbl_mode_{nullptr};
  QLabel * lbl_heading_{nullptr};
  QLabel * lbl_position_{nullptr};
  QLabel * lbl_temp_{nullptr};
  QLabel * lbl_runtime_{nullptr};
  QLabel * lbl_speed_{nullptr};
  QLabel * lbl_yaw_{nullptr};
  QLabel * lbl_pitch_{nullptr};
  QLabel * lbl_roll_{nullptr};
  QLabel * lbl_accel_{nullptr};

  QProgressBar * bar_batt_{nullptr};
  QTimer * ui_timer_{nullptr};

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_mode_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_battery_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_runtime_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_temperature_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_local_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_global_;

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  std::mutex mtx_;

  double roll_{0.0};
  double pitch_{0.0};
  double yaw_{0.0};

  double ax_{0.0};

  double speed_{0.0};
  double batt_{0.0};
  double battery_voltage_{0.0};
  double battery_current_{0.0};
  double temp_{0.0};
  double runtime_min_{0.0};

  double pos_x_{0.0};
  double pos_y_{0.0};
  double pos_z_{0.0};

  int mode_{0};
};

}  // namespace hugr_rviz_panel
