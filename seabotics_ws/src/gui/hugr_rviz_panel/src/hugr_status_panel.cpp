#include "hugr_rviz_panel/hugr_status_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFont>
#include <cmath>
#include <algorithm>

namespace hugr_rviz_panel
{

static QString fmt(double v, int d = 2)
{
  return QString::number(v, 'f', d);
}

static QString batteryStyleFor(int percent)
{
  QString color = "#4CAF50";
  if (percent < 20) color = "#E53935";
  else if (percent < 50) color = "#FBC02D";

  return QString(
    "QProgressBar { border: 1px solid #999; border-radius: 3px; text-align: center; height: 18px; }"
    "QProgressBar::chunk { background-color: %1; border-radius: 3px; }"
  ).arg(color);
}

HugrStatusPanel::HugrStatusPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QVBoxLayout(this);

  auto * title = new QLabel("Hugr-Primus - Status Panel");
  QFont tf; tf.setPointSize(16); tf.setBold(true);
  title->setFont(tf);
  title->setAlignment(Qt::AlignHCenter);
  root->addWidget(title);

  auto * grid = new QGridLayout();
  int r = 0;

  // Battery
  grid->addWidget(new QLabel("Battery:"), r, 0);
  bar_batt_ = new QProgressBar();
  bar_batt_->setRange(0,100);
  bar_batt_->setValue(0);
  bar_batt_->setFormat("0.0%");
  bar_batt_->setStyleSheet(batteryStyleFor(0));
  grid->addWidget(bar_batt_, r++, 1);

  // Battery temp
  grid->addWidget(new QLabel("Battery temp (°C):"), r, 0);
  lbl_temp_ = new QLabel("0.0");
  lbl_temp_->setStyleSheet("color: blue;");
  grid->addWidget(lbl_temp_, r++, 1);

  // Hull temp
  grid->addWidget(new QLabel("Hull Temperature (°C):"), r, 0);
  lbl_hull_temp_ = new QLabel("0.0");
  lbl_hull_temp_->setStyleSheet("color: blue;");
  grid->addWidget(lbl_hull_temp_, r++, 1);

  // Speed
  grid->addWidget(new QLabel("Speed (m/s):"), r, 0);
  lbl_speed_ = new QLabel("0.00");
  grid->addWidget(lbl_speed_, r++, 1);

  // Acceleration magnitude
  grid->addWidget(new QLabel("Acceleration (m/s^2):"), r, 0);
  lbl_accel_ = new QLabel("0.00");
  grid->addWidget(lbl_accel_, r++, 1);

  // Heading
  grid->addWidget(new QLabel("Heading (°):"), r, 0);
  lbl_heading_ = new QLabel("0.0");
  grid->addWidget(lbl_heading_, r++, 1);

  // Position
  grid->addWidget(new QLabel("Position (x,y,z):"), r, 0);
  lbl_position_ = new QLabel("x=0.00  y=0.00  z=0.00");
  grid->addWidget(lbl_position_, r++, 1);

  // Roll
  grid->addWidget(new QLabel("Roll (°):"), r, 0);
  lbl_roll_ = new QLabel("0.0");
  grid->addWidget(lbl_roll_, r++, 1);

  // Pitch
  grid->addWidget(new QLabel("Pitch (°):"), r, 0);
  lbl_pitch_ = new QLabel("0.0");
  grid->addWidget(lbl_pitch_, r++, 1);

  // Yaw
  grid->addWidget(new QLabel("Yaw (°):"), r, 0);
  lbl_yaw_ = new QLabel("0.0");
  grid->addWidget(lbl_yaw_, r++, 1);

  // ✅ 5G moved here: under Roll, above Mode
  grid->addWidget(new QLabel("5G Signal (dBm):"), r, 0);
  lbl_sig5g_ = new QLabel("0.0");
  grid->addWidget(lbl_sig5g_, r++, 1);

  // Mode
  grid->addWidget(new QLabel("Mode:"), r, 0);
  lbl_mode_ = new QLabel("AV");
  grid->addWidget(lbl_mode_, r++, 1);

  root->addLayout(grid);
  root->addStretch(1);
  setLayout(root);

  ui_timer_ = new QTimer(this);
  connect(ui_timer_, &QTimer::timeout, this, &HugrStatusPanel::onUiTimer);
  ui_timer_->start(100);
}

HugrStatusPanel::~HugrStatusPanel()
{
  stopRos();
}

void HugrStatusPanel::onInitialize()
{
  startRos();
}

void HugrStatusPanel::startRos()
{
  node_ = std::make_shared<rclcpp::Node>("hugr_status_panel");

  sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 10,
    std::bind(&HugrStatusPanel::imuCb, this, std::placeholders::_1));

  sub_twist_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&HugrStatusPanel::twistCb, this, std::placeholders::_1));

  sub_mode_ = node_->create_subscription<std_msgs::msg::String>(
    "/mode", 10,
    std::bind(&HugrStatusPanel::modeCb, this, std::placeholders::_1));

  sub_batt_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/battery_percent", 10,
    std::bind(&HugrStatusPanel::battCb, this, std::placeholders::_1));

  sub_batt_temp_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/battery_temp", 10,
    std::bind(&HugrStatusPanel::battTempCb, this, std::placeholders::_1));

  sub_pos_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "/position", 10,
    std::bind(&HugrStatusPanel::posCb, this, std::placeholders::_1));

  // NEW topics for new rows
  sub_sig5g_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/signal_5g_dbm", 10,
    std::bind(&HugrStatusPanel::sig5gCb, this, std::placeholders::_1));

  sub_hull_temp_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/hull_temp", 10,
    std::bind(&HugrStatusPanel::hullTempCb, this, std::placeholders::_1));

  exec_.add_node(node_);
  running_ = true;

  spin_thread_ = std::thread([this](){
    while (running_) {
      exec_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });
}

void HugrStatusPanel::stopRos()
{
  running_ = false;
  if (spin_thread_.joinable()) spin_thread_.join();
}

void HugrStatusPanel::imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double r, p, y;
  quatToRPY(msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w,
            r, p, y);

  std::lock_guard<std::mutex> l(mtx_);
  roll_  = r;
  pitch_ = p;
  yaw_   = y;

  ax_ = msg->linear_acceleration.x;
  ay_ = msg->linear_acceleration.y;
  az_ = msg->linear_acceleration.z;
}

void HugrStatusPanel::twistCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> l(mtx_);
  speed_ = std::hypot(msg->linear.x, msg->linear.y);
}

void HugrStatusPanel::modeCb(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> l(mtx_);
  mode_ = QString::fromStdString(msg->data).toUpper();
}

void HugrStatusPanel::battCb(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> l(mtx_);
  batt_ = msg->data;
}

void HugrStatusPanel::battTempCb(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> l(mtx_);
  temp_ = msg->data;
}

void HugrStatusPanel::posCb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> l(mtx_);
  pos_x_ = msg->point.x;
  pos_y_ = msg->point.y;
  pos_z_ = msg->point.z;
}

void HugrStatusPanel::sig5gCb(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> l(mtx_);
  sig5g_dbm_ = msg->data;
}

void HugrStatusPanel::hullTempCb(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> l(mtx_);
  hull_temp_ = msg->data;
}

void HugrStatusPanel::onUiTimer()
{
  double roll, pitch, yaw, speed, batt, temp, x, y, z;
  double ax, ay, az;
  double sig5g_dbm, hull_temp;
  QString mode;

  {
    std::lock_guard<std::mutex> l(mtx_);
    roll = roll_;
    pitch = pitch_;
    yaw = yaw_;

    ax = ax_;
    ay = ay_;
    az = az_;

    speed = speed_;
    batt = batt_;
    temp = temp_;

    x = pos_x_; y = pos_y_; z = pos_z_;

    sig5g_dbm = sig5g_dbm_;
    hull_temp = hull_temp_;

    mode = mode_;
  }

  // Acceleration magnitude |a|
  const double a_mag = std::sqrt(ax*ax + ay*ay + az*az);
  if (lbl_accel_) lbl_accel_->setText(QString::number(a_mag, 'f', 2));

  // Battery
  const double batt_clamped = std::max(0.0, std::min(100.0, batt));
  const int batt_int = static_cast<int>(std::round(batt_clamped));
  if (bar_batt_) {
    bar_batt_->setValue(batt_int);
    bar_batt_->setFormat(fmt(batt_clamped, 1) + "%");
    bar_batt_->setStyleSheet(batteryStyleFor(batt_int));
  }

  // Temps
  if (lbl_temp_) lbl_temp_->setText(fmt(temp, 1));
  if (lbl_hull_temp_) lbl_hull_temp_->setText(fmt(hull_temp, 1));

  // Speed
  if (lbl_speed_) lbl_speed_->setText(fmt(speed, 2));

  // Heading and position
  if (lbl_heading_) lbl_heading_->setText(fmt(yaw * 180.0 / M_PI, 1));

  if (lbl_position_) {
    lbl_position_->setText(QString("x=%1  y=%2  z=%3")
      .arg(fmt(x,2)).arg(fmt(y,2)).arg(fmt(z,2)));
  }

  // YPR
  if (lbl_roll_)   lbl_roll_->setText(fmt(yaw * 180.0 / M_PI, 1));
  if (lbl_pitch_) lbl_pitch_->setText(fmt(pitch * 180.0 / M_PI, 1));
  if (lbl_yaw_)  lbl_yaw_->setText(fmt(roll * 180.0 / M_PI, 1));

  // 5G signal
  if (lbl_sig5g_) lbl_sig5g_->setText(fmt(sig5g_dbm, 1));

  // -------- MODE HANDLING (Robust + Color coded) --------
  QString raw_mode = mode.trimmed().toLower().simplified();

  QString mode_text = "Unknown";
  QString mode_color = "white";

  if (raw_mode == "av") {
    mode_text  = "Kill Switch";
    mode_color = "red";
  } else if (raw_mode == "manuell" || raw_mode == "manual") {
    mode_text  = "Manual";
    mode_color = "#FFD700";
  } else if (raw_mode == "auto" || raw_mode == "ato" || raw_mode == "aut") {
    mode_text  = "Autonomous";
    mode_color = "limegreen";
  }

  if (lbl_mode_) {
    lbl_mode_->setText(mode_text);
    lbl_mode_->setStyleSheet(QString("QLabel { color: %1; font-weight: bold; }").arg(mode_color));
  }
}

void HugrStatusPanel::quatToRPY(double x, double y, double z, double w,
                               double & roll, double & pitch, double & yaw)
{
  const double sinr_cosp = 2.0 * (w*x + y*z);
  const double cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w*y - z*x);
  pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI/2.0, sinp) : std::asin(sinp);

  const double siny_cosp = 2.0 * (w*z + x*y);
  const double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace hugr_rviz_panel

PLUGINLIB_EXPORT_CLASS(hugr_rviz_panel::HugrStatusPanel, rviz_common::Panel)
