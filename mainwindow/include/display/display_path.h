
#pragma once
#include <Eigen/Dense>
#include <QColor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>

#include "virtual_display.h"
using namespace basic;
namespace Display {
class DisplayPath : public VirtualDisplay {
 private:
  QColor color_;
  QPolygonF path_points_;

 private:
  void drawPath(QPainter *painter);

  void computeBoundRect(const RobotPath &path);

 public:
  DisplayPath(const std::string &display_type, const int &z_value,
              std::string parent_name = "");
  ~DisplayPath();
  bool SetDisplayConfig(const std::string &config_name,
                        const std::any &config_data) override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  bool UpdateData(const std::any &data) override;
};
}  // namespace Display
