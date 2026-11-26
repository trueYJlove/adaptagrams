#ifndef LOCALROUTE_SRC_CHANNEL_NUDGER_H_
#define LOCALROUTE_SRC_CHANNEL_NUDGER_H_

#include <vector>

#include "localroute/local_route_solver.h"

namespace localroute {

class LocalOrthogonalRouter;

// Separates overlapping middle segments of independent electrical nets within
// local channel clearance bounds without modifying terminal pins or exit legs.
// In incremental mode (full_graph == false), only routes listed in
// mutable_conns may move; neighboring routes serve strictly as fixed anchors.
class ChannelNudger {
 public:
  static std::vector<Id> Nudge(LocalOrthogonalRouter* router,
                               const std::vector<Id>& mutable_conns,
                               bool full_graph);
};

}  // namespace localroute

#endif  // LOCALROUTE_SRC_CHANNEL_NUDGER_H_
