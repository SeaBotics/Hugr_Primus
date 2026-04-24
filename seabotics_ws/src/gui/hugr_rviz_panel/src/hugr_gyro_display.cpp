#include "hugr_rviz_panel/hugr_gyro_display.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#include <rclcpp/qos.hpp>
#include <OgreSceneManager.h>
#include <algorithm>
#include <cmath>

namespace hugr_rviz_panel
{

HugrGyroDisplay::HugrGyroDisplay()
{
  topic_property_ = new rviz_common::properties::RosTopicProperty(
    "IMU Topic",
    "/imu/data",
    QString::fromStdString(rosidl_generator_traits::name<sensor_msgs::msg::Imu>()),
    "sensor_msgs::msg::Imu topic to visualize",
    this,
    SLOT(updateTopic()));
}

HugrGyroDisplay::~HugrGyroDisplay()
{
  sub_.reset();
}

void HugrGyroDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();
  root_node_ = scene_node_->createChildSceneNode("hugr_gyro_root");
  ensureVisual();
  updateTopic();
}

void HugrGyroDisplay::onEnable()
{
  updateTopic();
}

void HugrGyroDisplay::onDisable()
{
  sub_.reset();
}

void HugrGyroDisplay::reset()
{
  rviz_common::Display::reset();
  {
    std::lock_guard<std::mutex> lk(mtx_);
    roll_ = pitch_ = yaw_ = 0.0;
  }
  have_new_ = true;
}

void HugrGyroDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
  if (!have_new_.exchange(false)) return;

  double r, p, y;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    r = roll_;
    p = pitch_;
    y = yaw_;
  }
  applyRings(r, p, y);
}

void HugrGyroDisplay::updateTopic()
{
  sub_.reset();
  if (!isEnabled()) return;

  const std::string topic = topic_property_->getStdString();
  if (topic.empty()) return;

  auto ros_node_abs = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abs) return;

  auto node = ros_node_abs->get_raw_node();

  sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    topic,
    rclcpp::SensorDataQoS(),
    std::bind(&HugrGyroDisplay::processMessage, this, std::placeholders::_1));
}

void HugrGyroDisplay::ensureVisual()
{
  if (yaw_ring_) return;

  auto * sm = context_->getSceneManager();

  // Uavhengige ringer: alle tre henger direkte på root.
  // Dette gjør at yaw, pitch og roll alltid vises samtidig.
  yaw_node_   = root_node_->createChildSceneNode("hugr_yaw_node");
  pitch_node_ = root_node_->createChildSceneNode("hugr_pitch_node");
  roll_node_  = root_node_->createChildSceneNode("hugr_roll_node");

  yaw_ring_   = std::make_unique<rviz_rendering::BillboardLine>(sm, yaw_node_);
  pitch_ring_ = std::make_unique<rviz_rendering::BillboardLine>(sm, pitch_node_);
  roll_ring_  = std::make_unique<rviz_rendering::BillboardLine>(sm, roll_node_);

  auto make_ring_xy = [&](std::unique_ptr<rviz_rendering::BillboardLine> & ring,
                          float r, float g, float b)
  {
    ring->clear();
    ring->setLineWidth(line_width_);
    ring->setColor(r, g, b, 1.0f);
    ring->setNumLines(1);
    ring->setMaxPointsPerLine(segments_ + 1);

    for (int i = 0; i <= segments_; ++i) {
      const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments_);
      const float cx = radius_ * static_cast<float>(std::cos(a));
      const float cy = radius_ * static_cast<float>(std::sin(a));
      ring->addPoint(Ogre::Vector3(cx, cy, 0.0f)); // sirkel i XY
    }
  };

  // Farger: yaw=rød, pitch=blå, roll=grønn
  make_ring_xy(yaw_ring_,   1.0f, 0.0f, 0.0f);
  make_ring_xy(pitch_ring_, 0.0f, 0.0f, 1.0f);
  make_ring_xy(roll_ring_,  0.0f, 1.0f, 0.0f);

  // Vi lar ringene være XY-sirkler, men "base-plan" legges inn i applyRings()
  applyRings(0.0, 0.0, 0.0);
}

void HugrGyroDisplay::processMessage(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  double r, p, y;
  quatToRPY(msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w,
            r, p, y);

  {
    std::lock_guard<std::mutex> lk(mtx_);
    roll_ = r;
    pitch_ = p;
    yaw_ = y;
  }
  have_new_ = true;
}

void HugrGyroDisplay::applyRings(double roll, double pitch, double yaw)
{
  if (!yaw_node_ || !pitch_node_ || !roll_node_) return;

  // Base-plan (hvordan ringene står i ro):
  // - yaw-ring:  XY -> XZ  (rotate +90° around X)
  // - pitch-ring: XY -> YZ (rotate +90° around Y)
  // - roll-ring:  XY       (identity)
  const Ogre::Quaternion base_yaw  (Ogre::Radian(static_cast<float>(M_PI_2)), Ogre::Vector3::UNIT_X);
  const Ogre::Quaternion base_pitch(Ogre::Radian(static_cast<float>(M_PI_2)), Ogre::Vector3::UNIT_Y);
  const Ogre::Quaternion base_roll (Ogre::Quaternion::IDENTITY);

  // Dynamisk rotasjon (gimbal): yaw ytterst, pitch midten, roll innerst
  const Ogre::Quaternion q_yaw  (Ogre::Radian(static_cast<float>(yaw)),   Ogre::Vector3::UNIT_Z);
  const Ogre::Quaternion q_pitch(Ogre::Radian(static_cast<float>(pitch)), Ogre::Vector3::UNIT_Y);
  const Ogre::Quaternion q_roll (Ogre::Radian(static_cast<float>(roll)),  Ogre::Vector3::UNIT_X);

  yaw_node_->setOrientation(q_yaw * base_yaw);
  pitch_node_->setOrientation(q_pitch * base_pitch);
  roll_node_->setOrientation(q_roll * base_roll);
}

void HugrGyroDisplay::quatToRPY(double x, double y, double z, double w,
                               double & roll, double & pitch, double & yaw)
{
  roll  = std::atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y));
  const double sinp = 2.0*(w*y - z*x);
  pitch = std::asin(std::clamp(sinp, -1.0, 1.0));
  yaw   = std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
}

}  // namespace hugr_rviz_panel

PLUGINLIB_EXPORT_CLASS(hugr_rviz_panel::HugrGyroDisplay, rviz_common::Display)
