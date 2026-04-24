#include <rviz_common/panel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <QVBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>

#include <chrono>
#include <memory>
#include <thread>

namespace hugr_rviz_panel
{

class HugrPIDPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit HugrPIDPanel(QWidget* parent = nullptr)
  : rviz_common::Panel(parent)
  {
    node_ = std::make_shared<rclcpp::Node>("hugr_pid_panel");

    auto* layout = new QVBoxLayout;

    kp_surge_ = createSpinBox("Kp_surge", layout);
    ki_surge_ = createSpinBox("Ki_surge", layout);

    kp_sway_ = createSpinBox("Kp_sway", layout);
    ki_sway_ = createSpinBox("Ki_sway", layout);

    kp_yaw_ = createSpinBox("Kp_yaw", layout);
    kd_yaw_ = createSpinBox("Kd_yaw", layout);

    auto* send_button = new QPushButton("Apply PID");
    layout->addWidget(send_button);

    connect(send_button, &QPushButton::clicked,
            this, &HugrPIDPanel::sendParameters);

    setLayout(layout);

    RCLCPP_INFO(node_->get_logger(), "HugrPIDPanel started");
  }

private:
  QDoubleSpinBox* createSpinBox(const QString& name, QVBoxLayout* layout)
  {
    auto* label = new QLabel(name);
    auto* box = new QDoubleSpinBox;

    box->setRange(-10000.0, 10000.0);
    box->setSingleStep(0.1);
    box->setDecimals(3);

    layout->addWidget(label);
    layout->addWidget(box);

    return box;
  }

  void sendParameters()
  {
    auto client =
      std::make_shared<rclcpp::SyncParametersClient>(node_, "velocity_node");

    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(node_->get_logger(), "velocity_node not available");
      return;
    }

    client->set_parameters({
      rclcpp::Parameter("Kp_surge", kp_surge_->value()),
      rclcpp::Parameter("Ki_surge", ki_surge_->value()),
      rclcpp::Parameter("Kp_sway", kp_sway_->value()),
      rclcpp::Parameter("Ki_sway", ki_sway_->value()),
      rclcpp::Parameter("Kp_yaw", kp_yaw_->value()),
      rclcpp::Parameter("Kd_yaw", kd_yaw_->value())
    });

    RCLCPP_INFO(node_->get_logger(), "PID updated from RViz panel");
  }

  rclcpp::Node::SharedPtr node_;

  QDoubleSpinBox* kp_surge_;
  QDoubleSpinBox* ki_surge_;
  QDoubleSpinBox* kp_sway_;
  QDoubleSpinBox* ki_sway_;
  QDoubleSpinBox* kp_yaw_;
  QDoubleSpinBox* kd_yaw_;
};

}  // namespace hugr_rviz_panel

PLUGINLIB_EXPORT_CLASS(hugr_rviz_panel::HugrPIDPanel, rviz_common::Panel)

#include "hugr_pid_panel.moc"
