#pragma once

#include <rviz_common/panel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <QTimer>

#include <mutex>
#include <thread>
#include <atomic>

namespace hugr_rviz_panel
{

class CompassWidget;

class HugrCompassPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit HugrCompassPanel(QWidget* parent = nullptr);
  ~HugrCompassPanel() override;
  void onInitialize() override;

private Q_SLOTS:
  void onUiTimer();

private:
  void startRos();
  void stopRos();
  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg);

  static void quatToYaw(double x, double y, double z, double w, double& yaw);

  // UI
  CompassWidget* compass_{nullptr};
  QTimer* ui_timer_{nullptr};

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  // Data
  std::mutex mtx_;
  double yaw_{0.0};   // rad
};

}  // namespace hugr_rviz_panel
