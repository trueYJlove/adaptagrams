#ifndef LOCALROUTE_INCLUDE_LOCALROUTE_LOCAL_ROUTE_SOLVER_H_
#define LOCALROUTE_INCLUDE_LOCALROUTE_LOCAL_ROUTE_SOLVER_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace localroute {

using Id = std::uint32_t;

struct Point {
  double x = 0.0;
  double y = 0.0;
};

inline bool operator==(Point a, Point b) { return a.x == b.x && a.y == b.y; }

struct Rect {
  double l = 0.0;
  double t = 0.0;
  double r = 0.0;
  double b = 0.0;

  Rect Pad(double d) const { return {l - d, t - d, r + d, b + d}; }

  bool Intersects(Rect a) const {
    return l <= a.r && a.l <= r && t <= a.b && a.t <= b;
  }

  bool Inside(Point p) const {
    return l < p.x && p.x < r && t < p.y && p.y < b;
  }
};

inline Rect Box(Point a, Point b) {
  return {std::min(a.x, b.x), std::min(a.y, b.y), std::max(a.x, b.x),
          std::max(a.y, b.y)};
}

enum Direction : unsigned {
  kUp = 1,
  kDown = 2,
  kLeft = 4,
  kRight = 8,
};

// Normalized coordinates in [0, 1]; shared electrical terminal.
struct Pin {
  Id id = 0;
  double x = 0.0;
  double y = 0.0;
  unsigned dirs = 0;
};

struct Block {
  Id id = 0;
  Rect rect;
  std::vector<Pin> pins;
};

struct Conn {
  Id id = 0;
  Id src = 0;
  Id sp = 0;
  Id dst = 0;
  Id dp = 0;
};

struct Config {
  double cell = 256.0;
  double padding = 250.0;
  double buffer = 8.0;
  double penalty = 50.0;
  double nudge_distance = 10.0;
  double max_nudge_distance = 24.0;
  unsigned retries = 3;
  unsigned budget = 120;
  bool settle_nudge = false;
  bool fast_paths = true;
  bool profile = false;
  bool reroute_third_party_blocked = false;
  bool enable_channel_nudging = true;
  double wire_overlap_penalty = 25.0;
};

enum class Status { kValid, kDisplayOnly, kFailed, kPending };

enum class Outcome {
  kNone,
  kCoincidentPins,
  kBuriedPin,
  kInvalidCandidate,
  kCollision,
  kObstacleBudget,
};

enum class RouteMethod { kNone, kReuse, kSimple, kNative };

struct Profile {
  double query_ms = 0.0;
  double setup_ms = 0.0;
  double graph_ms = 0.0;
  double search_ms = 0.0;
  double validation_ms = 0.0;
  double native_other_ms = 0.0;
  unsigned invalid_attempts = 0;
  unsigned collision_attempts = 0;
  unsigned capped_attempts = 0;
};

struct Route {
  Status status = Status::kFailed;
  std::vector<Point> points;
  unsigned calls = 0;
  unsigned retries = 0;
  unsigned window = 0;
  Outcome outcome = Outcome::kNone;
  // Pending routes may retain previously validated points and method for reuse.
  RouteMethod method = RouteMethod::kNone;
  bool user_modified = false;
  Profile profile;
};

struct OccupiedSegment {
  Point u;
  Point v;
  Id src_block = 0;
  Id src_pin = 0;
};

struct Request {
  const Config& cfg;
  const Conn& conn;
  const std::vector<const Block*>& obstacles;
  bool nudge = false;
  Profile* profile = nullptr;
  const std::vector<OccupiedSegment>* occupied_wires = nullptr;
};

class LocalRouteSolver {
 public:
  virtual ~LocalRouteSolver() = default;
  virtual const char* name() const = 0;
  virtual std::vector<Point> Solve(const Request& request) const = 0;
};

std::shared_ptr<const LocalRouteSolver> LazyHananAStarSolver();

}  // namespace localroute

#endif  // LOCALROUTE_INCLUDE_LOCALROUTE_LOCAL_ROUTE_SOLVER_H_
