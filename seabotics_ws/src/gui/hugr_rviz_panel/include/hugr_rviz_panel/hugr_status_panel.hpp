#pragma once

#include <rviz_common/panel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
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
  void twistCb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void modeCb(const std_msgs::msg::String::SharedPtr msg);

  // Fakeable topics
  void battCb(const std_msgs::msg::Float32::SharedPtr msg);
  void battTempCb(const std_msgs::msg::Float32::SharedPtr msg);
  void posCb(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  // NEW rows
  void sig5gCb(const std_msgs::msg::Float32::SharedPtr msg);     // dBm
  void hullTempCb(const std_msgs::msg::Float32::SharedPtr msg);  // °C

  static void quatToRPY(double x, double y, double z, double w,
                        double & roll, double & pitch, double & yaw);

  // ---------- UI ----------
  QLabel * lbl_mode_{nullptr};
  QLabel * lbl_heading_{nullptr};
  QLabel * lbl_position_{nullptr};
  QLabel * lbl_temp_{nullptr};
  QLabel * lbl_speed_{nullptr};
  QLabel * lbl_yaw_{nullptr};
  QLabel * lbl_pitch_{nullptr};
  QLabel * lbl_roll_{nullptr};

  // Acceleration magnitude |a| (m/s^2)
  QLabel * lbl_accel_{nullptr};

  // NEW rows UI
  QLabel * lbl_sig5g_{nullptr};      // dBm
  QLabel * lbl_hull_temp_{nullptr};  // °C

  QProgressBar * bar_batt_{nullptr};
  QTimer * ui_timer_{nullptr};

  // ---------- ROS ----------
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;

  // Battery %, battery temp, position
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_batt_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_batt_temp_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_pos_;

  // NEW subs
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_sig5g_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_hull_temp_;

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  // ---------- DATA (start = 0) ----------
  std::mutex mtx_;

  double roll_{0.0};
  double pitch_{0.0};
  double yaw_{0.0};

  double ax_{0.0};
  double ay_{0.0};
  double az_{0.0};

  double speed_{0.0};
  double batt_{0.0};   // percent
  double temp_{0.0};   // degC

  double pos_x_{0.0};
  double pos_y_{0.0};
  double pos_z_{0.0};

  // NEW rows data
  double sig5g_dbm_{0.0};
  double hull_temp_{0.0};

  QString mode_{"AV"};
};

}  // namespace hugr_rviz_panel
