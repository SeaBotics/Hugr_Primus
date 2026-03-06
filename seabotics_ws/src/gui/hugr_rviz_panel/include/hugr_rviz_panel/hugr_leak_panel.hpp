#ifndef HUGR_LEAK_PANEL_HPP_
#define HUGR_LEAK_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <QWidget>
#include <QPainter>

namespace hugr_rviz_panel
{

class HugrLeakPanel : public rviz_common::Panel
{
public:
  explicit HugrLeakPanel(QWidget *parent = nullptr);

protected:
  void onInitialize() override;
  void paintEvent(QPaintEvent *event) override;

private:
  void leakCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;

  // Nå bare 2 seksjoner
  float leak_levels_[2] = {0.0, 0.0};
};

}  // namespace hugr_rviz_panel

#endif
