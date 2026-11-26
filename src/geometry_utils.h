#ifndef LOCALROUTE_SRC_GEOMETRY_UTILS_H_
#define LOCALROUTE_SRC_GEOMETRY_UTILS_H_

#include <algorithm>

#include "localroute/local_route_solver.h"

namespace localroute {

inline double RectPointGap(Rect r, Point p) {
  const double dx = std::max({r.l - p.x, p.x - r.r, 0.0});
  const double dy = std::max({r.t - p.y, p.y - r.b, 0.0});
  return std::max(dx, dy);
}

inline double EffectiveObstacleBuffer(Rect obs_rect, Point a, Point b,
                                      double base_buffer) {
  const double g =
      std::min(RectPointGap(obs_rect, a), RectPointGap(obs_rect, b));
  if (g >= base_buffer) {
    return base_buffer;
  }
  return std::max(0.0, g * 0.5 - 0.5);
}

}  // namespace localroute

#endif  // LOCALROUTE_SRC_GEOMETRY_UTILS_H_
