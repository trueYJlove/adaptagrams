#ifndef LOCALROUTE_SRC_CHANNEL_NUDGER_H_
#define LOCALROUTE_SRC_CHANNEL_NUDGER_H_

#include <unordered_map>
#include <vector>

#include "localroute/local_route_solver.h"
#include "localroute/spatial_grid_index.h"

namespace localroute {

class LocalOrthogonalRouter;

// Separates overlapping middle segments of independent electrical nets within
// local channel clearance bounds without modifying terminal pins or exit legs.
class ChannelNudger {
 public:
  static std::vector<Id> Nudge(LocalOrthogonalRouter* router,
                               const std::vector<Id>& seed_conns,
                               bool full_graph);
};

}  // namespace localroute

#endif  // LOCALROUTE_SRC_CHANNEL_NUDGER_H_
