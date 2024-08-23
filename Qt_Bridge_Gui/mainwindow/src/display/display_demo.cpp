
#include "display/display_demo.h"
namespace Display {
DisplayDemo::DisplayDemo(const std::string &display_type, const int &z_value,
                         std::string parent_name)
    : VirtualDisplay(display_type, z_value, parent_name) {}
void DisplayDemo::paint(QPainter *painter,
                        const QStyleOptionGraphicsItem *option,
                        QWidget *widget) {
  drawFrame(painter);
}

DisplayDemo::~DisplayDemo() {}
bool DisplayDemo::UpdateData(const std::any &data) {
  try {

    update();
  } catch (const std::bad_any_cast &e) {
    std::cout << e.what() << '\n';
  }
  return true;
}
void DisplayDemo::drawFrame(QPainter *painter) {}
} // namespace Display
