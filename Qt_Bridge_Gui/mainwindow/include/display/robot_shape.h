
#pragma once
#include <Eigen/Dense>
#include <QColor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include "config/config_manager.h"
#include "virtual_display.h"
namespace Display {

class RobotShape : public VirtualDisplay {
 private:
  QPainterPath path_;
  RobotPose robot_pose_{0, 0, 0};
  QColor color_{0x1E90FF};
  float opacity_{0.5};

 private:
  void drawFrame(QPainter *painter);

 public:
  RobotShape(const std::string &display_type, const int &z_value,
             std::string parent_name = "");
  ~RobotShape();
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  bool UpdateData(const std::any &data) override;
};

}  // namespace Display
