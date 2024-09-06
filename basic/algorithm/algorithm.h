
#ifndef ROBOTALGORITHM_H
#define ROBOTALGORITHM_H
#include <math.h>

namespace basic {

// 角度转弧度
inline double deg2rad(double x) { return M_PI * x / 180.0; }
// 弧度转角度
inline double rad2deg(double x) { return 180.0 * x / M_PI; }

inline double normalize(double theta) {
  if (theta >= -M_PI && theta < M_PI)
    return theta;

  int multiplier = (int)(theta / (2 * M_PI));
  theta = theta - multiplier * 2 * M_PI;
  if (theta >= M_PI)
    theta -= 2 * M_PI;
  if (theta < -M_PI)
    theta += 2 * M_PI;
  return theta;
}

}  // namespace basic

#endif  // ROBOTALGORITHM_H
