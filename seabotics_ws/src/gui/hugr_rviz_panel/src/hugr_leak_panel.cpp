#include <QPainterPath>
#include "hugr_rviz_panel/hugr_leak_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

namespace hugr_rviz_panel
{

HugrLeakPanel::HugrLeakPanel(QWidget *parent)
  : rviz_common::Panel(parent)
{
  // NB: Ikke bruk getDisplayContext() her. Den kan være null i konstruktøren.
  setMinimumHeight(200);
}

void HugrLeakPanel::onInitialize()
{
  // Nå er DisplayContext satt
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
  if (msg->data.size() >= 4)
  {
    for (int i = 0; i < 4; i++)
      leak_levels_[i] = msg->data[i];

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

  // Dark background (fills whole panel)
  p.fillRect(rect(), QColor(28, 28, 28));

  // ============================================================
  // Keep boat UNSTRETCHED (uniform scale). Background grows only.
  // ============================================================
  const int DW = 900;   // design width
  const int DH = 320;   // design height
  const double sx = double(W) / double(DW);
  const double sy = double(H) / double(DH);
  const double s  = std::min(sx, sy);
  const double tx = (W - DW * s) * 0.5;
  const double ty = (H - DH * s) * 0.5;

  p.save();
  p.translate(tx, ty);
  p.scale(s, s);

  // ============================================================
  // Geometry (STRAIGHT NOSE like your drawing: flat top -> diagonal -> flat bottom)
  // ============================================================
  const int margin = 60;
  const int hullW  = DW - 2 * margin;
  const int hullH  = 110;
  const int y0     = DH/2 - hullH/2;

  const int xL = margin;
  const int xR = margin + hullW;

  // Nose (the "tip" you showed)
  const int noseTipOut = 40;                 // shorter tip                 // how far the tip goes left
  const int noseLen    = int(hullW * 0.18);  // where bottom starts (to the right)

  const int tipX = xL - noseTipOut;
  const int tipY = y0;

  const int bottomStartX = xL + noseLen;
  const int bottomY      = y0 + hullH;

  // Stern (simple; you can tune if you want later)
  const int sternTopX    = xR;          // stern straight down (top)          // top-right point
  const int sternBottomX = xR;          // stern straight down (bottom)          // bottom-right point (slight out)

  // Outer hull path (all LINE segments, no curves)
  QPainterPath hull;
  hull.moveTo(tipX, tipY);                  // tip (top-left)
  hull.lineTo(sternTopX, y0);               // flat top
  hull.lineTo(sternBottomX, bottomY);       // down/right at stern
  hull.lineTo(bottomStartX, bottomY);       // flat bottom
  hull.closeSubpath();                      // diagonal back to tip

  // Outline glow (draw twice)
  p.setPen(QPen(QColor(255,255,255,70), 7, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.setBrush(Qt::NoBrush);
  p.drawPath(hull);

  p.setPen(QPen(QColor(240,240,240), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.drawPath(hull);

  // ============================================================
  // Inner hull (inset) for fill/clipping
  // ============================================================
  const int inset = 3;  // tighter inner fill  // tighter inner fill
  QPainterPath inner;
  inner.moveTo(tipX + inset, tipY + inset);
  inner.lineTo(sternTopX - inset, y0 + inset);
  inner.lineTo(sternBottomX - inset, bottomY - inset);
  inner.lineTo(bottomStartX + inset, bottomY - inset);
  inner.closeSubpath();

// ============================================================
  // Fill sections (Section 1 reaches the tip because we clip to 'inner')
  // Section ratios: 1 bigger, 2 smaller
  // ============================================================
  p.save();
  p.setClipPath(inner);

  const double ratios[4] = {0.38, 0.14, 0.24, 0.24}; // 1 bigger, 2 smaller
  const int fillX0 = tipX - 200;                      // extend left for full fill                      // extend left for full fill                      // start BEFORE tip so section1 fills it
  const int fillX1 = sternBottomX + 200;              // extend right for full fill              // extend right for full fill                   // end at stern
  const int totalW = fillX1 - fillX0;

  int x = fillX0;
  for (int i = 0; i < 4; i++)
  {
    int wsec = int(totalW * ratios[i]);
    if (i == 3) wsec = fillX1 - x; // last takes remainder

    const float level = leak_levels_[i];
    QColor base;
    if (level < 0.3f)      base = QColor(235,235,235);
    else if (level < 0.7f) base = QColor(255,140,30);
    else                   base = QColor(200,30,30);

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

  // ============================================================
  // Section separators (clipped to inner)
  // ============================================================
  p.save();
  p.setClipPath(inner);
  p.setPen(QPen(QColor(20,20,20), 3));

  int sxline = fillX0;
  for (int k = 1; k < 4; k++)
  {
    sxline += int(totalW * ratios[k-1]);
    p.drawLine(sxline, y0, sxline, bottomY);
  }
  p.restore();

  // Bottom shadow
  p.setPen(QPen(QColor(0,0,0,130), 8, Qt::SolidLine, Qt::RoundCap));
  p.drawLine(xL + 30, bottomY + 12, xR - 20, bottomY + 12);

  p.restore(); // restore scale/translate
}






}  // namespace hugr_rviz_panel
