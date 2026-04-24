#include <QPainterPath>
#include "hugr_rviz_panel/hugr_leak_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <functional>

namespace hugr_rviz_panel
{

HugrLeakPanel::HugrLeakPanel(QWidget *parent)
  : rviz_common::Panel(parent)
{
  setMinimumHeight(200);
}

void HugrLeakPanel::onInitialize()
{
  auto ctx = getDisplayContext();
  if (!ctx) {
    RCLCPP_ERROR(rclcpp::get_logger("hugr_leak_panel"), "DisplayContext is null");
    return;
  }

  auto abs = ctx->getRosNodeAbstraction().lock();
  if (!abs) {
    RCLCPP_ERROR(rclcpp::get_logger("hugr_leak_panel"), "RosNodeAbstraction is null");
    return;
  }

  node_ = abs->get_raw_node();

  sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/sensors/water_leak_raw", 10,
    std::bind(&HugrLeakPanel::leakCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "HugrLeakPanel subscribed to /sensors/water_leak_raw");
}

void HugrLeakPanel::leakCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  leak_raw_ = std::clamp(msg->data, 0.0f, 1023.0f);
  update();
}

void HugrLeakPanel::paintEvent(QPaintEvent *event)
{
  Q_UNUSED(event);
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  const int W = width();
  const int H = height();

  p.fillRect(rect(), QColor(28, 28, 28));

  const int DW = 900;
  const int DH = 320;
  const double sx = double(W) / double(DW);
  const double sy = double(H) / double(DH);
  const double s  = std::min(sx, sy);
  const double tx = (W - DW * s) * 0.5;
  const double ty = (H - DH * s) * 0.5;

  p.save();
  p.translate(tx, ty);
  p.scale(s, s);

  const int margin = 60;
  const int hullW  = DW - 2 * margin;
  const int hullH  = 110;
  const int y0     = DH / 2 - hullH / 2;

  const int xL = margin;
  const int xR = margin + hullW;

  const int noseTipOut = 40;
  const int noseLen    = int(hullW * 0.18);

  const int tipX = xL - noseTipOut;
  const int tipY = y0;

  const int bottomStartX = xL + noseLen;
  const int bottomY      = y0 + hullH;

  const int sternTopX    = xR;
  const int sternBottomX = xR;

  QPainterPath hull;
  hull.moveTo(tipX, tipY);
  hull.lineTo(sternTopX, y0);
  hull.lineTo(sternBottomX, bottomY);
  hull.lineTo(bottomStartX, bottomY);
  hull.closeSubpath();

  p.setPen(QPen(QColor(255,255,255,70), 7, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.setBrush(Qt::NoBrush);
  p.drawPath(hull);

  p.setPen(QPen(QColor(240,240,240), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.drawPath(hull);

  const int inset = 3;
  QPainterPath inner;
  inner.moveTo(tipX + inset, tipY + inset);
  inner.lineTo(sternTopX - inset, y0 + inset);
  inner.lineTo(sternBottomX - inset, bottomY - inset);
  inner.lineTo(bottomStartX + inset, bottomY - inset);
  inner.closeSubpath();

  p.save();
  p.setClipPath(inner);

  QColor base;
  if (leak_raw_ > 700.0f) {
    base = QColor(200, 30, 30);
  } else if (leak_raw_ > 400.0f) {
    base = QColor(230, 150, 20);
  } else {
    base = QColor(235, 235, 235);
  }

  QLinearGradient g(tipX, y0, tipX, bottomY);
  g.setColorAt(0.0, base.lighter(135));
  g.setColorAt(0.55, base);
  g.setColorAt(1.0, base.darker(135));

  p.setPen(Qt::NoPen);
  p.setBrush(g);
  p.drawPath(inner);
  p.restore();

  p.setPen(QPen(QColor(0,0,0,130), 8, Qt::SolidLine, Qt::RoundCap));
  p.drawLine(xL + 30, bottomY + 12, xR - 20, bottomY + 12);

  p.setPen(QPen(QColor(240,240,240), 2));
  p.drawText(QRect(0, 20, DW, 40), Qt::AlignCenter,
             QString("Water leak raw: %1 / 1023").arg(leak_raw_, 0, 'f', 0));

  p.restore();
}

}  // namespace hugr_rviz_panel

PLUGINLIB_EXPORT_CLASS(hugr_rviz_panel::HugrLeakPanel, rviz_common::Panel)
