#include <QPainterPath>
#include "hugr_rviz_panel/hugr_leak_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <pluginlib/class_list_macros.hpp>

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
    RCLCPP_ERROR(rclcpp::get_logger("hugr_leak_panel"), "DisplayContext is null in onInitialize()");
    return;
  }

  auto weak_abs = ctx->getRosNodeAbstraction();
  auto abs = weak_abs.lock();
  if (!abs) {
    RCLCPP_ERROR(rclcpp::get_logger("hugr_leak_panel"), "RosNodeAbstraction is null");
    return;
  }

  node_ = abs->get_raw_node();
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("hugr_leak_panel"), "Raw node is null");
    return;
  }

  sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/leak/levels", 10,
    std::bind(&HugrLeakPanel::leakCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "HugrLeakPanel subscribed to /leak/levels");
}

void HugrLeakPanel::leakCallback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() >= 2)
  {
    leak_levels_[0] = msg->data[0];
    leak_levels_[1] = msg->data[1];
    update();
  }
}

void HugrLeakPanel::paintEvent(QPaintEvent *event)
{
  Q_UNUSED(event);
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  const int W = width();
  const int H = height();

  // Bakgrunn
  p.fillRect(rect(), QColor(28, 28, 28));

  // Hold skroget ustrukket
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

  // Geometri
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

  // Ytre skrog
  QPainterPath hull;
  hull.moveTo(tipX, tipY);
  hull.lineTo(sternTopX, y0);
  hull.lineTo(sternBottomX, bottomY);
  hull.lineTo(bottomStartX, bottomY);
  hull.closeSubpath();

  // Outline
  p.setPen(QPen(QColor(255,255,255,70), 7, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.setBrush(Qt::NoBrush);
  p.drawPath(hull);

  p.setPen(QPen(QColor(240,240,240), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.drawPath(hull);

  // Indre skrog
  const int inset = 3;
  QPainterPath inner;
  inner.moveTo(tipX + inset, tipY + inset);
  inner.lineTo(sternTopX - inset, y0 + inset);
  inner.lineTo(sternBottomX - inset, bottomY - inset);
  inner.lineTo(bottomStartX + inset, bottomY - inset);
  inner.closeSubpath();

  // Fyll 2 seksjoner, kun hvit/rød
  p.save();
  p.setClipPath(inner);

  const int fillX0 = tipX - 200;
  const int fillX1 = sternBottomX + 200;
  const int totalW = fillX1 - fillX0;

  int x = fillX0;
  for (int i = 0; i < 2; i++)
  {
    int wsec = (i == 0) ? totalW / 2 : (fillX1 - x);

    const float level = leak_levels_[i];
    QColor base = (level > 0.5f) ? QColor(200, 30, 30)
                                 : QColor(235, 235, 235);

    QLinearGradient g(x, y0, x, bottomY);
    g.setColorAt(0.0, base.lighter(135));
    g.setColorAt(0.55, base);
    g.setColorAt(1.0, base.darker(135));

    p.setPen(Qt::NoPen);
    p.setBrush(g);
    p.drawRect(x, y0, wsec, hullH);

    x += wsec;
  }

  p.restore();

  // Kun én separator i midten
  p.save();
  p.setClipPath(inner);
  p.setPen(QPen(QColor(20,20,20), 3));

  const int midLine = fillX0 + totalW / 2;
  p.drawLine(midLine, y0, midLine, bottomY);

  p.restore();

  // Skygge under
  p.setPen(QPen(QColor(0,0,0,130), 8, Qt::SolidLine, Qt::RoundCap));
  p.drawLine(xL + 30, bottomY + 12, xR - 20, bottomY + 12);

  p.restore();
}

}  // namespace hugr_rviz_panel

// PLUGINLIB_EXPORT_CLASS(hugr_rviz_panel::HugrLeakPanel, rviz_common::Panel)  // disabled: exported in hugr_leak_panel_plugin.cpp
