#pragma once

#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <OgreSceneNode.h>
#include <memory>
#include <mutex>
#include <atomic>

namespace hugr_rviz_panel
{

class HugrGyroDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  HugrGyroDisplay();
  ~HugrGyroDisplay() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void reset() override;

  // RViz render-loop (TRYGG plass å oppdatere OGRE)
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  void updateTopic();

private:
  void processMessage(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void ensureVisual();
  void applyRings(double roll, double pitch, double yaw);

  static void quatToRPY(double x, double y, double z, double w,
                        double & roll, double & pitch, double & yaw);

  rviz_common::properties::RosTopicProperty * topic_property_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;

  Ogre::SceneNode * root_node_{nullptr};
  Ogre::SceneNode * yaw_node_{nullptr};
  Ogre::SceneNode * pitch_node_{nullptr};
  Ogre::SceneNode * roll_node_{nullptr};

  std::unique_ptr<rviz_rendering::BillboardLine> yaw_ring_;
  std::unique_ptr<rviz_rendering::BillboardLine> pitch_ring_;
  std::unique_ptr<rviz_rendering::BillboardLine> roll_ring_;

  // Nyeste data fra callback (tråd-safe)
  std::mutex mtx_;
  double roll_{0.0};
  double pitch_{0.0};
  double yaw_{0.0};
  std::atomic<bool> have_new_{false};

  float radius_{1.0f};
  float line_width_{0.03f};
  int segments_{128};
};

}  // namespace hugr_rviz_panel
