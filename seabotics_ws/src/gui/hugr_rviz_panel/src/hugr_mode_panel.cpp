#include "hugr_rviz_panel/hugr_mode_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <QVBoxLayout>
#include <QLabel>
#include <QFont>

namespace hugr_rviz_panel
{

HugrModePanel::HugrModePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QVBoxLayout(this);

  auto * title = new QLabel("Hugr-Primus - Mode Panel");
  QFont tf;
  tf.setPointSize(16);
  tf.setBold(true);
  title->setFont(tf);
  title->setAlignment(Qt::AlignHCenter);
  root->addWidget(title);

  btn_kill_ = new QPushButton("KILL SWITCH");
  btn_manual_ = new QPushButton("MANUAL");
  btn_auto_ = new QPushButton("AUTONOMOUS");

  btn_kill_->setMinimumHeight(50);
  btn_manual_->setMinimumHeight(50);
  btn_auto_->setMinimumHeight(50);

  btn_kill_->setStyleSheet(
    "QPushButton { background-color: #E53935; color: white; font-weight: bold; font-size: 16px; }"
    "QPushButton:pressed { background-color: #B71C1C; }"
  );

  btn_manual_->setStyleSheet(
    "QPushButton { background-color: #FFD700; color: black; font-weight: bold; font-size: 16px; }"
    "QPushButton:pressed { background-color: #C9A800; }"
  );

  btn_auto_->setStyleSheet(
    "QPushButton { background-color: limegreen; color: white; font-weight: bold; font-size: 16px; }"
    "QPushButton:pressed { background-color: #228B22; }"
  );

  root->addWidget(btn_kill_);
  root->addWidget(btn_manual_);
  root->addWidget(btn_auto_);
  root->addStretch(1);

  setLayout(root);

  connect(btn_kill_, &QPushButton::clicked, this, &HugrModePanel::onKillClicked);
  connect(btn_manual_, &QPushButton::clicked, this, &HugrModePanel::onManualClicked);
  connect(btn_auto_, &QPushButton::clicked, this, &HugrModePanel::onAutoClicked);
}

HugrModePanel::~HugrModePanel()
{
  stopRos();
}

void HugrModePanel::onInitialize()
{
  startRos();
}

void HugrModePanel::startRos()
{
  node_ = std::make_shared<rclcpp::Node>("hugr_mode_panel");

  pub_mode_ = node_->create_publisher<std_msgs::msg::String>("/mode", 10);

  exec_.add_node(node_);
  running_ = true;

  spin_thread_ = std::thread([this]() {
    while (running_) {
      exec_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });
}

void HugrModePanel::stopRos()
{
  running_ = false;
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void HugrModePanel::publishMode(const std::string & mode)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (!pub_mode_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = mode;
  pub_mode_->publish(msg);
}

void HugrModePanel::onKillClicked()
{
  publishMode("av");
}

void HugrModePanel::onManualClicked()
{
  publishMode("manual");
}

void HugrModePanel::onAutoClicked()
{
  publishMode("auto");
}

}  // namespace hugr_rviz_panel

PLUGINLIB_EXPORT_CLASS(hugr_rviz_panel::HugrModePanel, rviz_common::Panel)


