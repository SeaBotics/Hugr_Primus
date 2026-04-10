#pragma once

#include <rviz_common/panel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <QPushButton>
#include <QTimer>

#include <mutex>
#include <thread>
#include <atomic>

namespace hugr_rviz_panel
{

class HugrModePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit HugrModePanel(QWidget * parent = nullptr);
  ~HugrModePanel() override;
  void onInitialize() override;

private Q_SLOTS:
  void onKillClicked();
  void onManualClicked();
  void onAutoClicked();

private:
  void startRos();
  void stopRos();
  void publishMode(const std::string & mode);

  QPushButton * btn_kill_{nullptr};
  QPushButton * btn_manual_{nullptr};
  QPushButton * btn_auto_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_;

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};
  std::mutex mtx_;
};

}  // namespace hugr_rviz_panel
