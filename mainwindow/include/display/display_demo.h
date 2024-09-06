
#pragma once
#include <Eigen/Dense>
#include <QColor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>

#include "virtual_display.h"
namespace Display {

class DisplayDemo : public VirtualDisplay {
 private:
  void drawFrame(QPainter *painter);

 public:
  DisplayDemo(const std::string &display_type, const int &z_value,
              std::string parent_name = "");
  ~DisplayDemo();
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  bool UpdateData(const std::any &data) override;
};

}  // namespace Display
