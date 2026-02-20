#include "hugr_rviz_panel/hugr_compass_panel.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <QVBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPaintEvent>
#include <QFont>

#include <cmath>
#include <algorithm>

namespace hugr_rviz_panel
{

class CompassWidget : public QWidget
{
public:
  explicit CompassWidget(QWidget* parent = nullptr)
  : QWidget(parent)
  {
    setMinimumSize(260, 260);
  }

  void setHeadingDeg(double heading_deg)
  {
    while (heading_deg < 0.0) heading_deg += 360.0;
    while (heading_deg >= 360.0) heading_deg -= 360.0;
    heading_deg_ = heading_deg;
    update();
  }

protected:
  void paintEvent(QPaintEvent*) override
  {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    const int w = width();
    const int h = height();
    const QPointF c(w * 0.5, h * 0.5);
    const double r = 0.45 * std::min(w, h);

    p.fillRect(rect(), palette().window());

    // Ring
    p.setPen(QPen(Qt::black, 2));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(c, r, r);

    auto drawLabel = [&](const QString& txt, double angle_deg, bool bold=false)
    {
      QFont f = p.font();
      f.setBold(bold);
      f.setPointSize(13);
      p.setFont(f);

      const double a = angle_deg * M_PI / 180.0;
      const QPointF pt(
        c.x() + r * 0.80 * std::cos(a),
        c.y() - r * 0.80 * std::sin(a)
      );

      const QRectF box(pt.x() - 18, pt.y() - 16, 36, 32);
      p.drawText(box, Qt::AlignCenter, txt);
    };

    // N/E/S/W faste posisjoner
    drawLabel("N", 90.0, true);
    drawLabel("E", 0.0);
    drawLabel("S", 270.0);
    drawLabel("W", 180.0);

    // Tick marks
    p.setPen(QPen(Qt::black, 1));
    for (int a = 0; a < 360; a += 30) {
      const double rad = a * M_PI / 180.0;
      const QPointF p1(c.x() + r * 0.92 * std::cos(rad), c.y() - r * 0.92 * std::sin(rad));
      const QPointF p2(c.x() + r * 1.00 * std::cos(rad), c.y() - r * 1.00 * std::sin(rad));
      p.drawLine(p1, p2);
    }

    // ✅ 0° = NORTH (opp). Qt 0° peker mot høyre => bruk (90 - heading)
    const double needle = (-heading_deg_) * M_PI / 180.0;
    const QPointF tip(
      c.x() + r * 0.85 * std::cos(needle),
      c.y() - r * 0.85 * std::sin(needle)
    );

    p.setPen(QPen(Qt::red, 3));
    p.drawLine(c, tip);

    p.setPen(Qt::NoPen);
    p.setBrush(Qt::black);
    p.drawEllipse(c, 4, 4);
  }

private:
  double heading_deg_{0.0};
};

HugrCompassPanel::HugrCompassPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  auto* root = new QVBoxLayout(this);

  auto* title = new QLabel("Hugr-Primus - Compass");
  QFont tf; tf.setPointSize(16); tf.setBold(true);
  title->setFont(tf);
  title->setAlignment(Qt::AlignHCenter);
  root->addWidget(title);

  compass_ = new CompassWidget(this);
  root->addWidget(compass_, 1);

  setLayout(root);

  ui_timer_ = new QTimer(this);
  connect(ui_timer_, &QTimer::timeout, this, &HugrCompassPanel::onUiTimer);
  ui_timer_->start(50);
}

HugrCompassPanel::~HugrCompassPanel()
{
  stopRos();
}

void HugrCompassPanel::onInitialize()
{
  startRos();
}

void HugrCompassPanel::startRos()
{
  node_ = std::make_shared<rclcpp::Node>("hugr_compass_panel");

  sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 10,
    std::bind(&HugrCompassPanel::imuCb, this, std::placeholders::_1));

  exec_.add_node(node_);
  running_ = true;

  spin_thread_ = std::thread([this](){
    while (running_) {
      exec_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });
}

void HugrCompassPanel::stopRos()
{
  running_ = false;
  if (spin_thread_.joinable()) spin_thread_.join();
}

void HugrCompassPanel::imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double yaw;
  quatToYaw(msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w,
            yaw);

  std::lock_guard<std::mutex> l(mtx_);
  yaw_ = yaw;
}

void HugrCompassPanel::onUiTimer()
{
  double yaw;
  {
    std::lock_guard<std::mutex> l(mtx_);
    yaw = yaw_;
  }

  // heading i grader: 0..360 (0 = North)
  double heading_deg = yaw * 180.0 / M_PI;
  while (heading_deg < 0.0) heading_deg += 360.0;
  while (heading_deg >= 360.0) heading_deg -= 360.0;

  if (compass_) compass_->setHeadingDeg(heading_deg);
}

void HugrCompassPanel::quatToYaw(double x, double y, double z, double w, double& yaw)
{
  const double siny_cosp = 2.0 * (w*z + x*y);
  const double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace hugr_rviz_panel

PLUGINLIB_EXPORT_CLASS(hugr_rviz_panel::HugrCompassPanel, rviz_common::Panel)
