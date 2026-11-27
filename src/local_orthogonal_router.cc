#include "localroute/local_orthogonal_router.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <locale>
#include <set>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <utility>

#include "channel_nudger.h"
#include "geometry_utils.h"

namespace localroute {
namespace {

using Clock = std::chrono::steady_clock;

struct Stopwatch {
  double* total;
  Clock::time_point start;

  explicit Stopwatch(double* out)
      : total(out), start(out ? Clock::now() : Clock::time_point()) {}

  ~Stopwatch() {
    if (total) {
      *total += std::chrono::duration<double, std::milli>(Clock::now() - start)
                    .count();
    }
  }
};

std::vector<Point> Elbow(Point a, Point b, bool horizontal) {
  if (a == b) {
    return {a};
  }
  if (a.x == b.x || a.y == b.y) {
    return {a, b};
  }
  return {a, horizontal ? Point{b.x, a.y} : Point{a.x, b.y}, b};
}

const Pin& FindPin(const Block& b, Id id) {
  for (const auto& p : b.pins) {
    if (p.id == id) {
      return p;
    }
  }
  throw std::invalid_argument("unknown pin");
}

Point PinPosition(Rect a, Pin p) {
  return {p.x == 1.0 ? a.r : a.l + p.x * (a.r - a.l),
          p.y == 1.0 ? a.b : a.t + p.y * (a.b - a.t)};
}

unsigned OrthoDir(Point a, Point b) {
  if (a == b) {
    return 0;
  }
  if (a.x == b.x) {
    return a.y < b.y ? kDown : kUp;
  }
  if (a.y == b.y) {
    return a.x < b.x ? kRight : kLeft;
  }
  return 0;
}

bool Crosses(Point a, Point b, Rect r) {
  const Rect s = Box(a, b);
  return s.r > r.l && s.l < r.r && s.b > r.t && s.t < r.b;
}

void CheckBox(Rect a) {
  if (!(a.l < a.r && a.t < a.b &&
        std::max({std::abs(a.l), std::abs(a.r), std::abs(a.t),
                  std::abs(a.b)}) <= 1e7)) {
    throw std::invalid_argument("invalid block rectangle");
  }
}

bool CanExit(Point end, Point next, Rect own, Rect buffered) {
  return !buffered.Inside(next) &&
         (own.Inside(end) || !Crosses(end, next, own));
}

bool FinitePoint(Point p) {
  return std::isfinite(p.x) && std::isfinite(p.y) &&
         std::max(std::abs(p.x), std::abs(p.y)) <= 5e7;
}

unsigned PrimaryDir(unsigned dirs) {
  if (dirs & kRight) {
    return kRight;
  }
  if (dirs & kLeft) {
    return kLeft;
  }
  if (dirs & kDown) {
    return kDown;
  }
  return kUp;
}

std::vector<Point> SimplifyOrtho(std::vector<Point> pts) {
  pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
  std::size_t count = 0;
  for (const Point v : pts) {
    if (count >= 2 && OrthoDir(pts[count - 2], pts[count - 1]) ==
                          OrthoDir(pts[count - 1], v)) {
      --count;
    }
    pts[count++] = v;
  }
  pts.resize(count);
  return pts;
}

std::vector<Point> CompactFallback(Point a, Point b, const Pin& sp,
                                   const Pin& dp) {
  if (a == b) {
    return {a, b};
  }
  const unsigned sd = PrimaryDir(sp.dirs);
  const unsigned dd = PrimaryDir(dp.dirs);
  if (a.x == b.x || a.y == b.y) {
    return {a, b};
  }
  if ((sd & (kLeft | kRight)) && (dd & (kLeft | kRight))) {
    const double mid_x = (a.x + b.x) * 0.5;
    return SimplifyOrtho({a, {mid_x, a.y}, {mid_x, b.y}, b});
  }
  if ((sd & (kUp | kDown)) && (dd & (kUp | kDown))) {
    const double mid_y = (a.y + b.y) * 0.5;
    return SimplifyOrtho({a, {a.x, mid_y}, {b.x, mid_y}, b});
  }
  return Elbow(a, b, (sd & (kLeft | kRight)) != 0);
}

double ComputeTerminalBufferFromIndex(
    const SpatialGridIndex& index, const std::unordered_map<Id, Block>& blocks,
    const Conn& c, Point pin_pt, double base_buffer) {
  double buf = base_buffer;
  for (Id id : index.Query(Box(pin_pt, pin_pt).Pad(base_buffer))) {
    if (id == c.src || id == c.dst) {
      continue;
    }
    const Rect r = blocks.at(id).rect;
    if (r.Inside(pin_pt)) {
      continue;
    }
    const double g = RectPointGap(r, pin_pt);
    if (g < base_buffer) {
      buf = std::min(buf, std::max(0.0, g * 0.5 - 0.5));
    }
  }
  return buf;
}

std::vector<Point> MakeSimulinkFallback(Point a, Point b, const Block& src,
                                        const Block& dst, const Pin& sp,
                                        const Pin& dp, const Config& cfg) {
  if (a == b) {
    return {a, b};
  }
  const unsigned sd = PrimaryDir(sp.dirs);
  const unsigned dd = PrimaryDir(dp.dirs);

  // Close/flush forward-facing aligned terminals connect directly in 2 points.
  if (sd == kRight && dd == kLeft && std::abs(a.y - b.y) <= 1e-3 &&
      b.x >= a.x - 1e-6) {
    return {a, b};
  }
  if (sd == kLeft && dd == kRight && std::abs(a.y - b.y) <= 1e-3 &&
      b.x <= a.x + 1e-6) {
    return {a, b};
  }
  if (sd == kDown && dd == kUp && std::abs(a.x - b.x) <= 1e-3 &&
      b.y >= a.y - 1e-6) {
    return {a, b};
  }
  if (sd == kUp && dd == kDown && std::abs(a.x - b.x) <= 1e-3 &&
      b.y <= a.y + 1e-6) {
    return {a, b};
  }

  // Only emit the 5-segment outer loop when src and dst actually overlap or
  // touch within buffer clearance; otherwise keep a compact L/Z fallback.
  if (!src.rect.Pad(cfg.buffer).Intersects(dst.rect.Pad(cfg.buffer))) {
    return CompactFallback(a, b, sp, dp);
  }

  const Rect u{
      std::min(src.rect.l, dst.rect.l), std::min(src.rect.t, dst.rect.t),
      std::max(src.rect.r, dst.rect.r), std::max(src.rect.b, dst.rect.b)};
  const double margin = std::max(cfg.buffer * 2.5, 20.0);

  const bool src_horiz = (sd & (kLeft | kRight)) != 0;
  const bool dst_horiz = (dd & (kLeft | kRight)) != 0;
  auto outer_x = [&](unsigned dir) {
    return (dir == kRight) ? (u.r + margin) : (u.l - margin);
  };
  auto outer_y = [&](unsigned dir) {
    return (dir == kDown) ? (u.b + margin) : (u.t - margin);
  };

  if (src_horiz && dst_horiz) {
    const double x_out = outer_x(sd);
    const double x_in = outer_x(dd);
    const double y_top = u.t - margin;
    const double y_bot = u.b + margin;
    const double y_mid = (a.y + b.y) * 0.5;
    const double y_rail =
        (std::abs(y_mid - y_top) <= std::abs(y_bot - y_mid)) ? y_top : y_bot;
    return SimplifyOrtho(
        {a, {x_out, a.y}, {x_out, y_rail}, {x_in, y_rail}, {x_in, b.y}, b});
  }
  if (!src_horiz && !dst_horiz) {
    const double y_out = outer_y(sd);
    const double y_in = outer_y(dd);
    const double x_left = u.l - margin;
    const double x_right = u.r + margin;
    const double x_mid = (a.x + b.x) * 0.5;
    const double x_rail =
        (std::abs(x_mid - x_left) <= std::abs(x_right - x_mid)) ? x_left
                                                                : x_right;
    return SimplifyOrtho(
        {a, {a.x, y_out}, {x_rail, y_out}, {x_rail, y_in}, {b.x, y_in}, b});
  }
  if (src_horiz && !dst_horiz) {
    const double x_out = outer_x(sd);
    const double y_in = outer_y(dd);
    return SimplifyOrtho({a, {x_out, a.y}, {x_out, y_in}, {b.x, y_in}, b});
  }
  const double y_out = outer_y(sd);
  const double x_in = outer_x(dd);
  return SimplifyOrtho({a, {a.x, y_out}, {x_in, y_out}, {x_in, b.y}, b});
}

bool SegmentsCollinearOverlap(Point a0, Point a1, Point b0, Point b1,
                              double match_eps) {
  if (std::abs(a0.y - a1.y) <= 1e-6 && std::abs(b0.y - b1.y) <= 1e-6) {
    if (std::abs(a0.y - b0.y) > match_eps) {
      return false;
    }
    const double ov0 = std::max(std::min(a0.x, a1.x), std::min(b0.x, b1.x));
    const double ov1 = std::min(std::max(a0.x, a1.x), std::max(b0.x, b1.x));
    return ov1 - ov0 > 1e-3;
  }
  if (std::abs(a0.x - a1.x) <= 1e-6 && std::abs(b0.x - b1.x) <= 1e-6) {
    if (std::abs(a0.x - b0.x) > match_eps) {
      return false;
    }
    const double ov0 = std::max(std::min(a0.y, a1.y), std::min(b0.y, b1.y));
    const double ov1 = std::min(std::max(a0.y, a1.y), std::max(b0.y, b1.y));
    return ov1 - ov0 > 1e-3;
  }
  return false;
}

bool RouteOverlapsOccupied(const std::vector<Point>& pts,
                           const std::vector<OccupiedSegment>& occupied,
                           double match_eps) {
  for (std::size_t j = 1; j < pts.size(); ++j) {
    for (const auto& occ : occupied) {
      if (SegmentsCollinearOverlap(pts[j - 1], pts[j], occ.u, occ.v,
                                   match_eps)) {
        return true;
      }
    }
  }
  return false;
}

std::vector<OccupiedSegment> CollectOccupiedForBlocks(
    const Conn& target, const std::vector<Id>& block_ids,
    const std::unordered_map<Id, Conn>& conns,
    const std::unordered_map<Id, std::vector<Id>>& adjacent,
    const std::unordered_map<Id, Route>& routes, std::size_t max_segments) {
  std::vector<Id> cand_conns;
  cand_conns.reserve(block_ids.size() * 4);
  for (Id bid : block_ids) {
    const auto ait = adjacent.find(bid);
    if (ait != adjacent.end()) {
      cand_conns.insert(cand_conns.end(), ait->second.begin(),
                        ait->second.end());
    }
  }
  std::sort(cand_conns.begin(), cand_conns.end());
  cand_conns.erase(std::unique(cand_conns.begin(), cand_conns.end()),
                   cand_conns.end());

  std::vector<OccupiedSegment> out;
  out.reserve(std::min<std::size_t>(max_segments, cand_conns.size() * 3));
  for (Id cid : cand_conns) {
    if (cid == target.id) {
      continue;
    }
    const auto cit = conns.find(cid);
    if (cit == conns.end()) {
      continue;
    }
    const Conn& other = cit->second;
    if (other.src == target.src && other.sp == target.sp) {
      continue;
    }
    const auto rit = routes.find(cid);
    if (rit == routes.end() || rit->second.status != Status::kValid ||
        rit->second.points.size() < 2) {
      continue;
    }
    const auto& pts = rit->second.points;
    for (std::size_t j = 1; j < pts.size(); ++j) {
      if (pts[j - 1] == pts[j]) {
        continue;
      }
      out.push_back({pts[j - 1], pts[j], other.src, other.sp});
      if (out.size() >= max_segments) {
        return out;
      }
    }
  }
  return out;
}

}  // namespace

LocalOrthogonalRouter::LocalOrthogonalRouter(
    Config cfg, std::shared_ptr<const LocalRouteSolver> solver)
    : cfg_(cfg),
      index_(cfg.cell),
      solver_(solver ? std::move(solver) : LazyHananAStarSolver()),
      segments_(cfg.cell) {
  if (!(cfg.cell >= 1.0 && cfg.padding > 0.0 && cfg.padding <= 1e5 &&
        cfg.buffer >= 0.0 && cfg.buffer <= 1e4 && cfg.penalty > 0.0 &&
        std::isfinite(cfg.penalty) && cfg.nudge_distance > 0.0 &&
        std::isfinite(cfg.nudge_distance) &&
        std::isfinite(cfg.max_nudge_distance) &&
        cfg.max_nudge_distance >= cfg.nudge_distance &&
        cfg.wire_overlap_penalty >= 0.0 &&
        std::isfinite(cfg.wire_overlap_penalty) && cfg.budget >= 2 &&
        cfg.retries <= 8)) {
    throw std::invalid_argument("invalid local router config");
  }
}

void LocalOrthogonalRouter::AddBlock(Block block) {
  CheckBox(block.rect);
  std::set<Id> seen;
  for (const auto& p : block.pins) {
    if (!p.id || p.id == 2147483647u ||
        !(p.x >= 0.0 && p.x <= 1.0 && p.y >= 0.0 && p.y <= 1.0) || !p.dirs ||
        (p.dirs & ~15u) || !seen.insert(p.id).second) {
      throw std::invalid_argument("invalid pin");
    }
  }
  index_.Insert(block.id, block.rect);
  const Id id = block.id;
  const Rect rect = block.rect;
  blocks_.emplace(id, std::move(block));
  if (cfg_.reroute_third_party_blocked) {
    InvalidateObstacle(id, rect);
  }
}

void LocalOrthogonalRouter::AddConnector(Conn conn) {
  (void)FindPin(blocks_.at(conn.src), conn.sp);
  (void)FindPin(blocks_.at(conn.dst), conn.dp);
  if (!conns_.emplace(conn.id, conn).second) {
    throw std::invalid_argument("duplicate connector");
  }
  adjacent_[conn.src].push_back(conn.id);
  if (conn.src != conn.dst) {
    adjacent_[conn.dst].push_back(conn.id);
  }
}

void LocalOrthogonalRouter::RemoveConnector(Id id) {
  const Conn c = conns_.at(id);
  auto unlink = [&](Id k) {
    auto& v = adjacent_.at(k);
    v.erase(std::find(v.begin(), v.end(), id));
  };
  unlink(c.src);
  if (c.src != c.dst) {
    unlink(c.dst);
  }
  ClearSegments(id);
  routes_.erase(id);
  dirty_.erase(id);
  conns_.erase(id);
}

void LocalOrthogonalRouter::RemoveBlock(Id id) {
  (void)blocks_.at(id);
  const auto ids = adjacent_[id];
  for (Id c : ids) {
    RemoveConnector(c);
  }
  adjacent_.erase(id);
  index_.Remove(id);
  blocks_.erase(id);
}

void LocalOrthogonalRouter::MoveBlocksBatch(
    const std::vector<std::pair<Id, Rect>>& moves) {
  for (const auto& m : moves) {
    (void)blocks_.at(m.first);
    CheckBox(m.second);
    index_.Check(m.second);
  }
  for (const auto& m : moves) {
    index_.Update(m.first, m.second);
    blocks_.at(m.first).rect = m.second;
  }
  for (const auto& m : moves) {
    const auto it = adjacent_.find(m.first);
    if (it != adjacent_.end()) {
      for (Id id : it->second) {
        Mark(id);
      }
    }
    if (cfg_.reroute_third_party_blocked) {
      InvalidateObstacle(m.first, blocks_.at(m.first).rect);
    }
  }
}

void LocalOrthogonalRouter::MoveBlock(Id id, double dx, double dy) {
  const Rect a = block(id).rect;
  MoveBlocksBatch({{id, {a.l + dx, a.t + dy, a.r + dx, a.b + dy}}});
}

void LocalOrthogonalRouter::set_reroute_third_party_blocked(bool enabled) {
  if (cfg_.reroute_third_party_blocked == enabled) {
    return;
  }
  cfg_.reroute_third_party_blocked = enabled;
  segments_ = SpatialGrid<std::uint64_t>(cfg_.cell);
  if (enabled) {
    for (const auto& entry : routes_) {
      if (entry.second.status == Status::kValid) {
        for (size_t j = 1; j < entry.second.points.size(); ++j) {
          segments_.Insert(
              (static_cast<std::uint64_t>(entry.first) << 32) | j,
              Box(entry.second.points[j - 1], entry.second.points[j]), true);
        }
      }
    }
  }
}

void LocalOrthogonalRouter::set_enable_channel_nudging(bool enabled) {
  cfg_.enable_channel_nudging = enabled;
}

std::vector<Id> LocalOrthogonalRouter::ApplyChannelNudging(
    const std::vector<Id>& mutable_conns, bool full_graph) {
  return ChannelNudger::Nudge(this, mutable_conns, full_graph);
}

bool LocalOrthogonalRouter::ValidateAndSimplifyRoute(
    const Conn& c, std::vector<Point>* points) const {
  if (!points) {
    return false;
  }
  const auto& src = blocks_.at(c.src);
  const auto& dst = blocks_.at(c.dst);
  const Pin sp = FindPin(src, c.sp);
  const Pin dp = FindPin(dst, c.dp);
  const Point a = PinPosition(src.rect, sp);
  const Point b = PinPosition(dst.rect, dp);
  const double src_buf =
      ComputeTerminalBufferFromIndex(index_, blocks_, c, a, cfg_.buffer);
  const double dst_buf =
      ComputeTerminalBufferFromIndex(index_, blocks_, c, b, cfg_.buffer);
  const Rect aa = src.rect.Pad(src_buf);
  const Rect bb = dst.rect.Pad(dst_buf);

  auto& p = *points;
  if (!std::all_of(p.begin(), p.end(), FinitePoint)) {
    return false;
  }
  p.erase(std::unique(p.begin(), p.end()), p.end());
  size_t count = 0;
  for (const Point v : p) {
    if (count >= 2 &&
        OrthoDir(p[count - 2], p[count - 1]) == OrthoDir(p[count - 1], v)) {
      --count;
    }
    p[count++] = v;
  }
  p.resize(count);

  if (p.size() < 2 || !(p.front() == a && p.back() == b) ||
      !(OrthoDir(a, p[1]) & sp.dirs) ||
      !(OrthoDir(b, p[p.size() - 2]) & dp.dirs)) {
    return false;
  }
  for (size_t j = 1; j < p.size(); ++j) {
    const Point u = p[j - 1];
    const Point v = p[j];
    if (!OrthoDir(u, v)) {
      return false;
    }
    for (Id id : index_.Query(Box(u, v).Pad(cfg_.buffer))) {
      const Rect raw = blocks_.at(id).rect;
      const double buf =
          (id == c.src)
              ? src_buf
              : (id == c.dst ? dst_buf
                             : EffectiveObstacleBuffer(raw, a, b, cfg_.buffer));
      const Rect rect = raw.Pad(buf);
      if (!Crosses(u, v, rect)) {
        continue;
      }
      if ((id == c.src && j == 1 && CanExit(u, v, src.rect, aa)) ||
          (id == c.dst && j + 1 == p.size() && CanExit(v, u, dst.rect, bb))) {
        continue;
      }
      return false;
    }
  }
  return true;
}

bool LocalOrthogonalRouter::ValidateNudgedMiddleSegment(
    const Conn& c, const std::vector<Point>& p, std::size_t seg_index) const {
  if (p.size() < 4 || seg_index < 1 || seg_index + 2 >= p.size()) {
    return false;
  }
  const auto& src = blocks_.at(c.src);
  const auto& dst = blocks_.at(c.dst);
  const Pin sp = FindPin(src, c.sp);
  const Pin dp = FindPin(dst, c.dp);
  const Point a = PinPosition(src.rect, sp);
  const Point b = PinPosition(dst.rect, dp);
  if (!(p.front() == a && p.back() == b)) {
    return false;
  }
  if (seg_index == 1 && !(OrthoDir(a, p[1]) & sp.dirs)) {
    return false;
  }
  if (seg_index + 2 == p.size() - 1 &&
      !(OrthoDir(b, p[p.size() - 2]) & dp.dirs)) {
    return false;
  }
  for (std::size_t j = seg_index; j <= seg_index + 2; ++j) {
    if (!OrthoDir(p[j - 1], p[j])) {
      return false;
    }
  }

  bool has_src_buf = false;
  double src_buf = cfg_.buffer;
  Rect aa = src.rect;
  auto get_src_buf = [&]() {
    if (!has_src_buf) {
      src_buf =
          ComputeTerminalBufferFromIndex(index_, blocks_, c, a, cfg_.buffer);
      aa = src.rect.Pad(src_buf);
      has_src_buf = true;
    }
    return src_buf;
  };
  bool has_dst_buf = false;
  double dst_buf = cfg_.buffer;
  Rect bb = dst.rect;
  auto get_dst_buf = [&]() {
    if (!has_dst_buf) {
      dst_buf =
          ComputeTerminalBufferFromIndex(index_, blocks_, c, b, cfg_.buffer);
      bb = dst.rect.Pad(dst_buf);
      has_dst_buf = true;
    }
    return dst_buf;
  };

  for (std::size_t j = seg_index; j <= seg_index + 2; ++j) {
    const Point u = p[j - 1];
    const Point v = p[j];
    const std::vector<Id> ids = index_.Query(Box(u, v).Pad(cfg_.buffer));
    if (j == 1 && !has_src_buf) {
      for (Id id : ids) {
        if (id == c.src || id == c.dst) {
          continue;
        }
        const Rect r = blocks_.at(id).rect;
        if (r.Inside(a)) {
          continue;
        }
        const double g = RectPointGap(r, a);
        if (g < cfg_.buffer) {
          src_buf = std::min(src_buf, std::max(0.0, g * 0.5 - 0.5));
        }
      }
      aa = src.rect.Pad(src_buf);
      has_src_buf = true;
    }
    if (j + 1 == p.size() && !has_dst_buf) {
      for (Id id : ids) {
        if (id == c.src || id == c.dst) {
          continue;
        }
        const Rect r = blocks_.at(id).rect;
        if (r.Inside(b)) {
          continue;
        }
        const double g = RectPointGap(r, b);
        if (g < cfg_.buffer) {
          dst_buf = std::min(dst_buf, std::max(0.0, g * 0.5 - 0.5));
        }
      }
      bb = dst.rect.Pad(dst_buf);
      has_dst_buf = true;
    }
    for (Id id : ids) {
      const Rect raw = blocks_.at(id).rect;
      double buf = 0.0;
      if (id == c.src) {
        buf = get_src_buf();
      } else if (id == c.dst) {
        buf = get_dst_buf();
      } else {
        buf = EffectiveObstacleBuffer(raw, a, b, cfg_.buffer);
      }
      const Rect rect = raw.Pad(buf);
      if (!Crosses(u, v, rect)) {
        continue;
      }
      if ((id == c.src && j == 1 && CanExit(u, v, src.rect, aa)) ||
          (id == c.dst && j + 1 == p.size() && CanExit(v, u, dst.rect, bb))) {
        continue;
      }
      return false;
    }
  }
  return true;
}

std::vector<Point> LocalOrthogonalRouter::Preview(Id id) const {
  const auto& c = conns_.at(id);
  const auto& a = blocks_.at(c.src);
  const auto& b = blocks_.at(c.dst);
  return Elbow(PinPosition(a.rect, FindPin(a, c.sp)),
               PinPosition(b.rect, FindPin(b, c.dp)), true);
}

void LocalOrthogonalRouter::Mark(Id id) {
  dirty_.insert(id);
  routes_[id].status = Status::kPending;
}

void LocalOrthogonalRouter::InvalidateObstacle(Id block_id, Rect rect) {
  rect = rect.Pad(cfg_.buffer);
  for (std::uint64_t key : segments_.Query(rect)) {
    const Id id = static_cast<Id>(key >> 32);
    const auto& c = conns_.at(id);
    if (c.src == block_id || c.dst == block_id || dirty_.count(id)) {
      continue;
    }
    const auto& p = routes_.at(id).points;
    const auto j = static_cast<std::uint32_t>(key);
    if (Crosses(p[j - 1], p[j], rect)) {
      Mark(id);
    }
  }
}

void LocalOrthogonalRouter::ClearSegments(Id id) {
  if (!cfg_.reroute_third_party_blocked) {
    return;
  }
  const auto it = routes_.find(id);
  if (it != routes_.end() && it->second.method != RouteMethod::kNone) {
    for (size_t j = 1; j < it->second.points.size(); ++j) {
      segments_.Remove((static_cast<std::uint64_t>(id) << 32) | j);
    }
  }
}

void LocalOrthogonalRouter::Publish(Id id, Route route) {
  if (cfg_.reroute_third_party_blocked) {
    ClearSegments(id);
    if (route.status == Status::kValid) {
      for (size_t j = 1; j < route.points.size(); ++j) {
        segments_.Insert((static_cast<std::uint64_t>(id) << 32) | j,
                         Box(route.points[j - 1], route.points[j]), true);
      }
    }
  }
  routes_[id] = std::move(route);
  dirty_.erase(id);
}

Route LocalOrthogonalRouter::SolveOne(const Conn& c, bool settled,
                                      bool force_auto_route,
                                      bool allow_congestion) const {
  const auto& src = blocks_.at(c.src);
  const auto& dst = blocks_.at(c.dst);
  const Pin sp = FindPin(src, c.sp);
  const Pin dp = FindPin(dst, c.dp);
  const Point a = PinPosition(src.rect, sp);
  const Point b = PinPosition(dst.rect, dp);
  Route out;
  if (a == b) {
    out.outcome = Outcome::kCoincidentPins;
    return out;
  }
  out.points = MakeSimulinkFallback(a, b, src, dst, sp, dp, cfg_);
  out.status = Status::kDisplayOnly;
  Profile* profile = cfg_.profile ? &out.profile : nullptr;
  auto query = [&](Rect rect) {
    Stopwatch clock(profile ? &profile->query_ms : nullptr);
    return index_.Query(rect);
  };
  double src_buf = cfg_.buffer;
  double dst_buf = cfg_.buffer;
  bool any_pin_pinched = false;
  // Only a terminal buried inside another block's solid body has no legal exit;
  // narrow buffer gaps are handled dynamically via shrunk obstacle buffers.
  auto inspect_terminal = [&](Point pt, Id own_id, double* out_buf) {
    for (Id id : query(Box(pt, pt).Pad(cfg_.buffer))) {
      if (id == own_id) {
        continue;
      }
      const Rect r = blocks_.at(id).rect;
      if (r.Inside(pt)) {
        return true;
      }
      if (id != c.src && id != c.dst) {
        const double g = RectPointGap(r, pt);
        if (g < cfg_.buffer) {
          any_pin_pinched = true;
          *out_buf = std::min(*out_buf, std::max(0.0, g * 0.5 - 0.5));
        }
      }
    }
    return false;
  };
  if (inspect_terminal(a, c.src, &src_buf) ||
      inspect_terminal(b, c.dst, &dst_buf)) {
    out.outcome = Outcome::kBuriedPin;
    return out;
  }
  const Rect aa = src.rect.Pad(src_buf);
  const Rect bb = dst.rect.Pad(dst_buf);

  std::set<Id> required{c.src, c.dst};
  auto accept = [&](std::vector<Point> p, RouteMethod method) {
    Stopwatch clock(profile ? &profile->validation_ms : nullptr);
    auto invalid = [&] {
      out.outcome = Outcome::kInvalidCandidate;
      if (profile && method == RouteMethod::kNative) {
        ++profile->invalid_attempts;
      }
      return false;
    };
    if (!std::all_of(p.begin(), p.end(), FinitePoint)) {
      return invalid();
    }
    p.erase(std::unique(p.begin(), p.end()), p.end());
    // Merge redundant collinear points before checking the terminal exits.
    if (method != RouteMethod::kNative) {
      size_t count = 0;
      for (const Point v : p) {
        if (count >= 2 &&
            OrthoDir(p[count - 2], p[count - 1]) == OrthoDir(p[count - 1], v)) {
          --count;
        }
        p[count++] = v;
      }
      p.resize(count);
    }
    if (p.size() < 2 || !(p.front() == a && p.back() == b) ||
        !(OrthoDir(a, p[1]) & sp.dirs) ||
        !(OrthoDir(b, p[p.size() - 2]) & dp.dirs)) {
      return invalid();
    }
    bool ok = true;
    for (size_t j = 1; j < p.size(); ++j) {
      const Point u = p[j - 1];
      const Point v = p[j];
      if (!OrthoDir(u, v)) {
        return invalid();
      }
      for (Id id : index_.Query(Box(u, v).Pad(cfg_.buffer))) {
        const Rect raw = blocks_.at(id).rect;
        double buf = cfg_.buffer;
        if (any_pin_pinched) {
          buf = (id == c.src)
                    ? src_buf
                    : (id == c.dst
                           ? dst_buf
                           : EffectiveObstacleBuffer(raw, a, b, cfg_.buffer));
        }
        const Rect rect = raw.Pad(buf);
        if (!Crosses(u, v, rect)) {
          continue;
        }
        if ((id == c.src && j == 1 && CanExit(u, v, src.rect, aa)) ||
            (id == c.dst && j + 1 == p.size() && CanExit(v, u, dst.rect, bb))) {
          continue;
        }
        if (method != RouteMethod::kNative) {
          return false;
        }
        required.insert(id);
        ok = false;
      }
    }
    if (!ok) {
      out.outcome = Outcome::kCollision;
      if (profile) {
        ++profile->collision_attempts;
      }
      return false;
    }
    out.status = Status::kValid;
    out.outcome = Outcome::kNone;
    out.method = method;
    out.points = std::move(p);
    return true;
  };

  const auto old_it = routes_.find(c.id);
  const bool has_old = old_it != routes_.end() &&
                       old_it->second.points.size() >= 2 &&
                       old_it->second.method != RouteMethod::kNone;

  auto try_translation_reuse = [&](bool preserve_user_mod) {
    if (!has_old) {
      return false;
    }
    const auto& p = old_it->second.points;
    const double dx = a.x - p.front().x;
    const double dy = a.y - p.front().y;
    if (dx == b.x - p.back().x && dy == b.y - p.back().y) {
      auto moved = p;
      for (auto& v : moved) {
        v = {v.x + dx, v.y + dy};
      }
      moved.front() = a;
      moved.back() = b;
      if (accept(std::move(moved), RouteMethod::kReuse)) {
        out.user_modified = preserve_user_mod;
        return true;
      }
    }
    return false;
  };

  auto try_endpoint_patch_reuse = [&](bool preserve_user_mod) {
    if (!has_old) {
      return false;
    }
    const auto& p = old_it->second.points;
    if (p.size() < 3) {
      return false;
    }
    auto repair = [&](Point end, Point old_end, Point bend) {
      return end == old_end ? bend
                            : (old_end.y == bend.y ? Point{bend.x, end.y}
                                                   : Point{end.x, bend.y});
    };
    const Point first = repair(a, p.front(), p[1]);
    const Point last = repair(b, p.back(), p[p.size() - 2]);
    std::vector<Point> patch{a, first};
    if (p.size() == 3) {
      patch.push_back(p[1]);
    } else {
      patch.insert(patch.end(), p.begin() + 2, p.end() - 2);
    }
    patch.push_back(last);
    patch.push_back(b);
    if (accept(std::move(patch), RouteMethod::kReuse)) {
      out.user_modified = preserve_user_mod;
      return true;
    }
    return false;
  };

  // Minimum disturbance: respect hand-edited segments ahead of Simple L/Z.
  if (!force_auto_route && has_old && old_it->second.user_modified) {
    if (try_translation_reuse(true) || try_endpoint_patch_reuse(true)) {
      return out;
    }
  }

  const double wire_match_eps = std::min(1.5, cfg_.nudge_distance * 0.25);
  std::vector<OccupiedSegment> precollected_occupied;
  if (force_auto_route && cfg_.wire_overlap_penalty > 0.0) {
    const std::vector<Id> nearby_blocks =
        query(Box(a, b).Pad(cfg_.padding + cfg_.buffer));
    precollected_occupied = CollectOccupiedForBlocks(c, nearby_blocks, conns_,
                                                     adjacent_, routes_, 128);
  }
  auto accept_simple = [&](std::vector<Point> p) {
    if (force_auto_route && !precollected_occupied.empty() &&
        RouteOverlapsOccupied(p, precollected_occupied, wire_match_eps)) {
      return false;
    }
    return accept(std::move(p), RouteMethod::kSimple);
  };

  if (cfg_.fast_paths &&
      (force_auto_route || !(settled && cfg_.settle_nudge))) {
    // 1. Whole-polyline translation reuse.
    if (!force_auto_route && try_translation_reuse(false)) {
      return out;
    }
    // 2. Clean 2/3-segment L and Z Simple candidates (eliminates stale hooks).
    if (accept_simple(Elbow(a, b, true)) ||
        (a.x != b.x && a.y != b.y && accept_simple(Elbow(a, b, false)))) {
      return out;
    }
    if (a.y != b.y && (sp.dirs & (kLeft | kRight)) &&
        (dp.dirs & (kLeft | kRight))) {
      for (double x :
           {(a.x + b.x) * 0.5, std::clamp((a.x + b.x) * 0.5, aa.l, aa.r),
            std::clamp((a.x + b.x) * 0.5, bb.l, bb.r)}) {
        if (accept_simple({a, {x, a.y}, {x, b.y}, b})) {
          return out;
        }
      }
    }
    if (a.x != b.x && (sp.dirs & (kUp | kDown)) && (dp.dirs & (kUp | kDown))) {
      for (double y :
           {(a.y + b.y) * 0.5, std::clamp((a.y + b.y) * 0.5, aa.t, aa.b),
            std::clamp((a.y + b.y) * 0.5, bb.t, bb.b)}) {
        if (accept_simple({a, {a.x, y}, {b.x, y}, b})) {
          return out;
        }
      }
    }
    // 3. Fallback: patch endpoints onto previous middle segments when Simple is
    //    blocked by obstacles.
    if (!force_auto_route && try_endpoint_patch_reuse(false)) {
      return out;
    }
  }

  const Rect bounds = Box(a, b);
  for (unsigned attempt = 0;
       attempt <= cfg_.retries && required.size() <= cfg_.budget; ++attempt) {
    std::vector<const Block*> obs;
    {
      Stopwatch clock(profile ? &profile->query_ms : nullptr);
      auto ids = index_.Query(
          bounds.Pad(std::ldexp(cfg_.padding, attempt) + cfg_.buffer));
      ids.insert(ids.end(), required.begin(), required.end());
      std::sort(ids.begin(), ids.end());
      ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
      if (ids.size() > cfg_.budget) {
        if (profile) {
          ++profile->capped_attempts;
        }
        const double vx = b.x - a.x;
        const double vy = b.y - a.y;
        const double den = vx * vx + vy * vy;
        auto priority = [&](Id id) {
          const Rect r = blocks_.at(id).rect;
          const double x = (r.l + r.r) * 0.5 - a.x;
          const double y = (r.t + r.b) * 0.5 - a.y;
          const double k = std::clamp((x * vx + y * vy) / den, 0.0, 1.0);
          return std::make_tuple(!required.count(id),
                                 std::hypot(x - k * vx, y - k * vy), id);
        };
        std::vector<std::tuple<bool, double, Id>> keys;
        keys.reserve(ids.size());
        for (Id id : ids) {
          keys.push_back(priority(id));
        }
        std::nth_element(keys.begin(), keys.begin() + cfg_.budget, keys.end());
        ids.clear();
        for (unsigned i = 0; i < cfg_.budget; ++i) {
          ids.push_back(std::get<2>(keys[i]));
        }
        std::sort(ids.begin(), ids.end());
      }
      for (Id id : ids) {
        obs.push_back(&blocks_.at(id));
      }
    }
    out.window =
        std::max<unsigned>(out.window, static_cast<unsigned>(obs.size()));
    out.retries = attempt;
    ++out.calls;

    const std::vector<OccupiedSegment>* occ_ptr = nullptr;
    std::vector<OccupiedSegment> local_occupied;
    if (force_auto_route && cfg_.wire_overlap_penalty > 0.0) {
      occ_ptr =
          precollected_occupied.empty() ? nullptr : &precollected_occupied;
    } else if (allow_congestion && cfg_.wire_overlap_penalty > 0.0 &&
               obs.size() <= 60) {
      std::vector<Id> obs_ids;
      obs_ids.reserve(obs.size());
      for (const Block* b_obs : obs) {
        obs_ids.push_back(b_obs->id);
      }
      local_occupied =
          CollectOccupiedForBlocks(c, obs_ids, conns_, adjacent_, routes_, 128);
      if (!local_occupied.empty()) {
        occ_ptr = &local_occupied;
      }
    }

    if (accept(solver_->Solve({cfg_, c, obs, settled && cfg_.settle_nudge,
                               profile, occ_ptr}),
               RouteMethod::kNative)) {
      return out;
    }
  }
  if (required.size() > cfg_.budget) {
    out.outcome = Outcome::kObstacleBudget;
  }
  return out;
}

std::vector<Id> LocalOrthogonalRouter::RerouteAffected(
    const std::vector<Id>& moved, bool settled) {
  auto ids = dirty_;
  for (Id id : moved) {
    (void)blocks_.at(id);
    const auto it = adjacent_.find(id);
    if (it != adjacent_.end()) {
      ids.insert(it->second.begin(), it->second.end());
    }
  }
  for (Id id : ids) {
    Publish(id, SolveOne(conns_.at(id), settled, /*force_auto_route=*/false,
                         /*allow_congestion=*/false));
  }
  if (cfg_.enable_channel_nudging && !ids.empty()) {
    (void)ApplyChannelNudging({ids.begin(), ids.end()}, false);
  }
  return {ids.begin(), ids.end()};
}

void LocalOrthogonalRouter::RouteConnector(Id id) {
  Publish(id, SolveOne(conns_.at(id), false, /*force_auto_route=*/false,
                       /*allow_congestion=*/false));
  if (cfg_.enable_channel_nudging) {
    (void)ApplyChannelNudging({id}, false);
  }
}

void LocalOrthogonalRouter::MarkConnector(Id id) {
  (void)conns_.at(id);
  Mark(id);
}

std::vector<Id> LocalOrthogonalRouter::UpdateBlockPins(
    Id block_id, Rect new_rect, std::vector<Pin> new_pins) {
  (void)blocks_.at(block_id);
  CheckBox(new_rect);
  index_.Check(new_rect);
  std::set<Id> seen;
  for (const auto& p : new_pins) {
    if (!p.id || p.id == 2147483647u ||
        !(p.x >= 0.0 && p.x <= 1.0 && p.y >= 0.0 && p.y <= 1.0) || !p.dirs ||
        (p.dirs & ~15u) || !seen.insert(p.id).second) {
      throw std::invalid_argument("invalid pin");
    }
  }

  index_.Update(block_id, new_rect);
  Block& blk = blocks_.at(block_id);
  blk.rect = new_rect;
  blk.pins = std::move(new_pins);

  std::vector<Id> adj;
  if (const auto ait = adjacent_.find(block_id); ait != adjacent_.end()) {
    adj = ait->second;
  }
  std::vector<Id> surviving;
  surviving.reserve(adj.size());
  for (Id cid : adj) {
    const auto cit = conns_.find(cid);
    if (cit == conns_.end()) {
      continue;
    }
    const Conn& c = cit->second;
    if ((c.src == block_id && !seen.count(c.sp)) ||
        (c.dst == block_id && !seen.count(c.dp))) {
      RemoveConnector(cid);
    } else {
      Mark(cid);
      surviving.push_back(cid);
    }
  }
  if (cfg_.reroute_third_party_blocked) {
    InvalidateObstacle(block_id, new_rect);
  }
  std::sort(surviving.begin(), surviving.end());
  surviving.erase(std::unique(surviving.begin(), surviving.end()),
                  surviving.end());
  return surviving;
}

void LocalOrthogonalRouter::RestoreRoute(Id conn_id, Route route) {
  (void)conns_.at(conn_id);
  Publish(conn_id, std::move(route));
}

bool LocalOrthogonalRouter::MoveRouteSegment(Id conn_id, std::size_t seg_index,
                                             double new_coord,
                                             std::vector<Id>* changed_out) {
  if (!std::isfinite(new_coord)) {
    return false;
  }
  const auto cit = conns_.find(conn_id);
  const auto rit = routes_.find(conn_id);
  if (cit == conns_.end() || rit == routes_.end() ||
      rit->second.points.size() < 2) {
    return false;
  }
  const Conn base_conn = cit->second;
  const std::vector<Point>& base_pts = rit->second.points;
  if (seg_index + 1 >= base_pts.size()) {
    return false;
  }
  const Point base_u = base_pts[seg_index];
  const Point base_v = base_pts[seg_index + 1];
  const unsigned base_dir = OrthoDir(base_u, base_v);
  if (!base_dir) {
    return false;
  }
  const bool base_is_vert = (base_dir & (kUp | kDown)) != 0;
  const double old_coord = base_is_vert ? base_u.x : base_u.y;
  const double base_k0 = std::min(base_is_vert ? base_u.y : base_u.x,
                                  base_is_vert ? base_v.y : base_v.x);
  const double base_k1 = std::max(base_is_vert ? base_u.y : base_u.x,
                                  base_is_vert ? base_v.y : base_v.x);
  const double base_split_coord = 0.5 * (base_k0 + base_k1);
  const bool base_is_end_split =
      (base_pts.size() > 2) &&
      (seg_index == 0 || seg_index + 2 == base_pts.size());

  // Collect same-(src, sp) fan-out siblings sharing a collinear overlapping
  // segment so shared trunks move together and keep solder joints intact.
  struct SiblingSeg {
    Id cid = 0;
    std::size_t seg_idx = 0;
    bool is_end_split = false;
  };
  std::vector<SiblingSeg> siblings;
  if (const auto ait = adjacent_.find(base_conn.src); ait != adjacent_.end()) {
    for (Id sib_id : ait->second) {
      if (sib_id == conn_id) {
        continue;
      }
      const auto scit = conns_.find(sib_id);
      const auto srit = routes_.find(sib_id);
      if (scit == conns_.end() || srit == routes_.end() ||
          scit->second.src != base_conn.src ||
          scit->second.sp != base_conn.sp || srit->second.points.size() < 2) {
        continue;
      }
      const auto& spts = srit->second.points;
      bool found_seg = false;
      std::size_t best_j = 0;
      bool best_mid = false;
      double best_ov = 0.0;
      for (std::size_t j = 0; j + 1 < spts.size(); ++j) {
        const unsigned sdir = OrthoDir(spts[j], spts[j + 1]);
        if (!sdir) {
          continue;
        }
        const bool s_vert = (sdir & (kUp | kDown)) != 0;
        if (s_vert != base_is_vert) {
          continue;
        }
        const double sc = s_vert ? spts[j].x : spts[j].y;
        if (std::abs(sc - old_coord) > 1e-3) {
          continue;
        }
        const double sk0 = std::min(s_vert ? spts[j].y : spts[j].x,
                                    s_vert ? spts[j + 1].y : spts[j + 1].x);
        const double sk1 = std::max(s_vert ? spts[j].y : spts[j].x,
                                    s_vert ? spts[j + 1].y : spts[j + 1].x);
        const double ov = std::min(base_k1, sk1) - std::max(base_k0, sk0);
        if (ov > 1e-3) {
          const bool is_mid =
              (spts.size() >= 4 && j >= 1 && j + 2 < spts.size());
          if (!found_seg || (is_mid && !best_mid) ||
              (is_mid == best_mid && ov > best_ov)) {
            found_seg = true;
            best_j = j;
            best_mid = is_mid;
            best_ov = ov;
          }
        }
      }
      if (found_seg) {
        const bool sib_end_split =
            (spts.size() > 2) && (best_j == 0 || best_j + 2 == spts.size());
        siblings.push_back({sib_id, best_j, sib_end_split});
      }
    }
  }

  auto apply_move_one = [&](Id target_cid, std::size_t target_seg_idx,
                            bool has_split_override,
                            double split_override) -> bool {
    const auto t_cit = conns_.find(target_cid);
    const auto t_rit = routes_.find(target_cid);
    if (t_cit == conns_.end() || t_rit == routes_.end() ||
        t_rit->second.points.size() < 2) {
      return false;
    }
    const Conn& conn = t_cit->second;
    const auto& src = blocks_.at(conn.src);
    const auto& dst = blocks_.at(conn.dst);
    const Pin sp = FindPin(src, conn.sp);
    const Pin dp = FindPin(dst, conn.dp);
    const Point a = PinPosition(src.rect, sp);
    const Point b = PinPosition(dst.rect, dp);
    const std::vector<Point>& old_pts = t_rit->second.points;
    if (target_seg_idx + 1 >= old_pts.size()) {
      return false;
    }
    const Point u = old_pts[target_seg_idx];
    const Point v = old_pts[target_seg_idx + 1];
    const unsigned dir = OrthoDir(u, v);
    if (!dir) {
      return false;
    }
    const bool is_vert = (dir & (kUp | kDown)) != 0;
    std::vector<Point> cand;

    if (old_pts.size() >= 4 && target_seg_idx >= 1 &&
        target_seg_idx + 2 < old_pts.size()) {
      cand = old_pts;
      if (is_vert) {
        cand[target_seg_idx].x = new_coord;
        cand[target_seg_idx + 1].x = new_coord;
      } else {
        cand[target_seg_idx].y = new_coord;
        cand[target_seg_idx + 1].y = new_coord;
      }
    } else if (old_pts.size() == 2) {
      if (is_vert) {
        if ((sp.dirs & (kLeft | kRight)) && (dp.dirs & (kLeft | kRight))) {
          cand = {a, {new_coord, a.y}, {new_coord, b.y}, b};
        } else {
          const double span = std::abs(b.y - a.y);
          double y_exit = a.y;
          double y_enter = b.y;
          if (b.y > a.y + 4.0 && (sp.dirs & kDown) && (dp.dirs & kUp)) {
            const double stub = std::min(20.0, std::max(2.0, span * 0.25));
            y_exit = a.y + stub;
            y_enter = b.y - stub;
          } else if (a.y > b.y + 4.0 && (sp.dirs & kUp) && (dp.dirs & kDown)) {
            const double stub = std::min(20.0, std::max(2.0, span * 0.25));
            y_exit = a.y - stub;
            y_enter = b.y + stub;
          } else {
            const double stub = std::max(cfg_.buffer + 4.0, 12.0);
            y_exit =
                a.y + ((sp.dirs & kUp) && !(sp.dirs & kDown) ? -stub : stub);
            y_enter =
                b.y + ((dp.dirs & kDown) && !(dp.dirs & kUp) ? stub : -stub);
          }
          cand = {a,
                  {a.x, y_exit},
                  {new_coord, y_exit},
                  {new_coord, y_enter},
                  {b.x, y_enter},
                  b};
        }
      } else {
        if ((sp.dirs & (kUp | kDown)) && (dp.dirs & (kUp | kDown))) {
          cand = {a, {a.x, new_coord}, {b.x, new_coord}, b};
        } else {
          const double span = std::abs(b.x - a.x);
          double x_exit = a.x;
          double x_enter = b.x;
          if (b.x > a.x + 4.0 && (sp.dirs & kRight) && (dp.dirs & kLeft)) {
            const double stub = std::min(20.0, std::max(2.0, span * 0.25));
            x_exit = a.x + stub;
            x_enter = b.x - stub;
          } else if (a.x > b.x + 4.0 && (sp.dirs & kLeft) &&
                     (dp.dirs & kRight)) {
            const double stub = std::min(20.0, std::max(2.0, span * 0.25));
            x_exit = a.x - stub;
            x_enter = b.x + stub;
          } else {
            const double stub = std::max(cfg_.buffer + 4.0, 12.0);
            x_exit =
                a.x + ((sp.dirs & kLeft) && !(sp.dirs & kRight) ? -stub : stub);
            x_enter =
                b.x + ((dp.dirs & kRight) && !(dp.dirs & kLeft) ? stub : -stub);
          }
          cand = {a,
                  {x_exit, a.y},
                  {x_exit, new_coord},
                  {x_enter, new_coord},
                  {x_enter, b.y},
                  b};
        }
      }
    } else if (target_seg_idx == 0) {
      cand.push_back(old_pts.front());
      if (is_vert) {
        const double split_y =
            has_split_override ? split_override : 0.5 * (u.y + v.y);
        cand.push_back({u.x, split_y});
        cand.push_back({new_coord, split_y});
        cand.push_back({new_coord, v.y});
      } else {
        const double split_x =
            has_split_override ? split_override : 0.5 * (u.x + v.x);
        cand.push_back({split_x, u.y});
        cand.push_back({split_x, new_coord});
        cand.push_back({v.x, new_coord});
      }
      for (std::size_t k = 2; k < old_pts.size(); ++k) {
        cand.push_back(old_pts[k]);
      }
    } else {
      for (std::size_t k = 0; k + 2 < old_pts.size(); ++k) {
        cand.push_back(old_pts[k]);
      }
      if (is_vert) {
        const double split_y =
            has_split_override ? split_override : 0.5 * (u.y + v.y);
        cand.push_back({new_coord, u.y});
        cand.push_back({new_coord, split_y});
        cand.push_back({v.x, split_y});
      } else {
        const double split_x =
            has_split_override ? split_override : 0.5 * (u.x + v.x);
        cand.push_back({u.x, new_coord});
        cand.push_back({split_x, new_coord});
        cand.push_back({split_x, v.y});
      }
      cand.push_back(old_pts.back());
    }

    cand = SimplifyOrtho(std::move(cand));
    if (cand.size() < 2) {
      return false;
    }
    for (std::size_t j = 1; j < cand.size(); ++j) {
      if (!OrthoDir(cand[j - 1], cand[j])) {
        return false;
      }
    }

    Route updated = t_rit->second;
    updated.method = RouteMethod::kReuse;
    updated.user_modified = true;
    updated.outcome = Outcome::kNone;
    std::vector<Point> validated = cand;
    if (ValidateAndSimplifyRoute(conn, &validated)) {
      updated.points = std::move(validated);
      updated.status = Status::kValid;
    } else {
      updated.points = std::move(cand);
      updated.status = Status::kDisplayOnly;
    }
    Publish(target_cid, std::move(updated));
    return true;
  };

  if (!apply_move_one(conn_id, seg_index, false, 0.0)) {
    return false;
  }
  if (changed_out) {
    changed_out->clear();
    changed_out->push_back(conn_id);
  }
  for (const SiblingSeg& sib : siblings) {
    const bool share_split = base_is_end_split && sib.is_end_split;
    if (apply_move_one(sib.cid, sib.seg_idx, share_split, base_split_coord) &&
        changed_out) {
      changed_out->push_back(sib.cid);
    }
  }
  return true;
}

std::vector<Id> LocalOrthogonalRouter::AutoRouteConnectors(
    const std::vector<Id>& conn_ids) {
  std::set<Id> targets;
  for (Id cid : conn_ids) {
    if (conns_.count(cid)) {
      targets.insert(cid);
    }
  }
  if (targets.empty()) {
    return {};
  }
  for (Id cid : targets) {
    auto rit = routes_.find(cid);
    if (rit != routes_.end()) {
      rit->second.user_modified = false;
    }
  }
  const std::vector<Id> changed(targets.begin(), targets.end());
  for (Id cid : changed) {
    Publish(cid, SolveOne(conns_.at(cid), /*settled=*/true,
                          /*force_auto_route=*/true,
                          /*allow_congestion=*/true));
  }
  if (cfg_.enable_channel_nudging) {
    const std::vector<Id> nudged = ApplyChannelNudging(changed, false);
    targets.insert(nudged.begin(), nudged.end());
  }
  return {targets.begin(), targets.end()};
}

std::vector<Point> LocalOrthogonalRouter::ComputeSolderJoints(
    const std::vector<Id>& conn_ids) const {
  if (conn_ids.size() < 2) {
    return {};
  }
  std::vector<Id> unique_ids = conn_ids;
  std::sort(unique_ids.begin(), unique_ids.end());
  unique_ids.erase(std::unique(unique_ids.begin(), unique_ids.end()),
                   unique_ids.end());

  struct NetConn {
    std::uint64_t key = 0;
    Id cid = 0;
    const std::vector<Point>* pts = nullptr;
  };
  std::vector<NetConn> valid_conns;
  valid_conns.reserve(unique_ids.size());
  for (Id cid : unique_ids) {
    const auto cit = conns_.find(cid);
    const auto rit = routes_.find(cid);
    if (cit == conns_.end() || rit == routes_.end() ||
        rit->second.status != Status::kValid || rit->second.points.size() < 2) {
      continue;
    }
    const Conn& c = cit->second;
    const std::uint64_t key = (static_cast<std::uint64_t>(c.src) << 32) | c.sp;
    valid_conns.push_back({key, cid, &rit->second.points});
  }
  if (valid_conns.size() < 2) {
    return {};
  }
  std::sort(valid_conns.begin(), valid_conns.end(),
            [](const NetConn& a, const NetConn& b) {
              return a.key != b.key ? a.key < b.key : a.cid < b.cid;
            });

  struct HorizSeg {
    double y = 0.0;
    double x0 = 0.0;
    double x1 = 0.0;
    std::size_t idx = 0;
  };
  struct VertSeg {
    double x = 0.0;
    double y0 = 0.0;
    double y1 = 0.0;
    std::size_t idx = 0;
  };
  struct WireGeom {
    const std::vector<Point>* pts = nullptr;
    std::vector<HorizSeg> h_segs;
    std::vector<VertSeg> v_segs;
  };

  constexpr double kEps = 1e-3;
  auto near_pt = [](Point a, Point b) {
    return std::abs(a.x - b.x) <= kEps && std::abs(a.y - b.y) <= kEps;
  };
  auto is_terminal = [&](const std::vector<Point>& pts, Point p) {
    return near_pt(p, pts.front()) || near_pt(p, pts.back());
  };

  auto out_dir_horiz = [&](const WireGeom& wg, const HorizSeg& s, double px,
                           unsigned away_dir) -> unsigned {
    if (s.x0 + kEps < px && px < s.x1 - kEps) {
      return away_dir;
    }
    const auto& pts = *wg.pts;
    const Point p{px, s.y};
    std::size_t v_idx = s.idx;
    std::size_t inside_idx = s.idx + 1;
    if (near_pt(pts[s.idx + 1], p)) {
      v_idx = s.idx + 1;
      inside_idx = s.idx;
    }
    if (v_idx == 0 || v_idx + 1 == pts.size()) {
      return 0;
    }
    const std::size_t other_idx =
        (inside_idx == v_idx + 1) ? (v_idx - 1) : (v_idx + 1);
    return OrthoDir(pts[v_idx], pts[other_idx]);
  };

  auto out_dir_vert = [&](const WireGeom& wg, const VertSeg& s, double py,
                          unsigned away_dir) -> unsigned {
    if (s.y0 + kEps < py && py < s.y1 - kEps) {
      return away_dir;
    }
    const auto& pts = *wg.pts;
    const Point p{s.x, py};
    std::size_t v_idx = s.idx;
    std::size_t inside_idx = s.idx + 1;
    if (near_pt(pts[s.idx + 1], p)) {
      v_idx = s.idx + 1;
      inside_idx = s.idx;
    }
    if (v_idx == 0 || v_idx + 1 == pts.size()) {
      return 0;
    }
    const std::size_t other_idx =
        (inside_idx == v_idx + 1) ? (v_idx - 1) : (v_idx + 1);
    return OrthoDir(pts[v_idx], pts[other_idx]);
  };

  std::vector<Point> raw_joints;
  std::vector<WireGeom> wires;
  for (std::size_t g_start = 0; g_start < valid_conns.size();) {
    const std::uint64_t group_key = valid_conns[g_start].key;
    std::size_t g_end = g_start + 1;
    while (g_end < valid_conns.size() && valid_conns[g_end].key == group_key) {
      ++g_end;
    }
    if (g_end - g_start < 2) {
      g_start = g_end;
      continue;
    }

    wires.clear();
    wires.reserve(g_end - g_start);
    for (std::size_t k = g_start; k < g_end; ++k) {
      WireGeom wg;
      wg.pts = valid_conns[k].pts;
      const auto& pts = *wg.pts;
      for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const Point& p0 = pts[i];
        const Point& p1 = pts[i + 1];
        if (std::abs(p0.y - p1.y) <= kEps && std::abs(p0.x - p1.x) > kEps) {
          wg.h_segs.push_back({0.5 * (p0.y + p1.y), std::min(p0.x, p1.x),
                               std::max(p0.x, p1.x), i});
        } else if (std::abs(p0.x - p1.x) <= kEps &&
                   std::abs(p0.y - p1.y) > kEps) {
          wg.v_segs.push_back({0.5 * (p0.x + p1.x), std::min(p0.y, p1.y),
                               std::max(p0.y, p1.y), i});
        }
      }
      wires.push_back(std::move(wg));
    }

    const Id src_id = static_cast<Id>(group_key >> 32);
    const Id sp_id = static_cast<Id>(group_key & 0xffffffffu);
    const Point src_pin = PinPosition(blocks_.at(src_id).rect,
                                      FindPin(blocks_.at(src_id), sp_id));

    auto push_joint = [&](Point p) {
      if (!near_pt(p, src_pin)) {
        raw_joints.push_back(p);
      }
    };

    for (std::size_t i = 0; i < wires.size(); ++i) {
      for (std::size_t j = i + 1; j < wires.size(); ++j) {
        const WireGeom& wi = wires[i];
        const WireGeom& wj = wires[j];

        // 1. T-junctions: interior vertex of one wire strictly inside a
        //    segment of the other wire.
        auto check_vertices = [&](const WireGeom& src_w,
                                  const WireGeom& seg_w) {
          const auto& pts = *src_w.pts;
          for (std::size_t k = 1; k + 1 < pts.size(); ++k) {
            const Point p = pts[k];
            if (is_terminal(*seg_w.pts, p)) {
              continue;
            }
            bool hit = false;
            for (const auto& h : seg_w.h_segs) {
              if (std::abs(p.y - h.y) <= kEps && h.x0 + kEps < p.x &&
                  p.x < h.x1 - kEps) {
                hit = true;
                break;
              }
            }
            if (!hit) {
              for (const auto& v : seg_w.v_segs) {
                if (std::abs(p.x - v.x) <= kEps && v.y0 + kEps < p.y &&
                    p.y < v.y1 - kEps) {
                  hit = true;
                  break;
                }
              }
            }
            if (hit) {
              push_joint(p);
            }
          }
        };
        check_vertices(wi, wj);
        check_vertices(wj, wi);

        // 2. Shared collinear segment divergence endpoints.
        for (const auto& h1 : wi.h_segs) {
          for (const auto& h2 : wj.h_segs) {
            if (std::abs(h1.y - h2.y) > kEps) {
              continue;
            }
            const double ox0 = std::max(h1.x0, h2.x0);
            const double ox1 = std::min(h1.x1, h2.x1);
            if (ox1 - ox0 <= kEps) {
              continue;
            }
            const double my = 0.5 * (h1.y + h2.y);
            const unsigned d1_l = out_dir_horiz(wi, h1, ox0, kLeft);
            const unsigned d2_l = out_dir_horiz(wj, h2, ox0, kLeft);
            if (d1_l && d2_l && d1_l != d2_l) {
              push_joint({ox0, my});
            }
            const unsigned d1_r = out_dir_horiz(wi, h1, ox1, kRight);
            const unsigned d2_r = out_dir_horiz(wj, h2, ox1, kRight);
            if (d1_r && d2_r && d1_r != d2_r) {
              push_joint({ox1, my});
            }
          }
        }
        for (const auto& v1 : wi.v_segs) {
          for (const auto& v2 : wj.v_segs) {
            if (std::abs(v1.x - v2.x) > kEps) {
              continue;
            }
            const double oy0 = std::max(v1.y0, v2.y0);
            const double oy1 = std::min(v1.y1, v2.y1);
            if (oy1 - oy0 <= kEps) {
              continue;
            }
            const double mx = 0.5 * (v1.x + v2.x);
            const unsigned d1_u = out_dir_vert(wi, v1, oy0, kUp);
            const unsigned d2_u = out_dir_vert(wj, v2, oy0, kUp);
            if (d1_u && d2_u && d1_u != d2_u) {
              push_joint({mx, oy0});
            }
            const unsigned d1_d = out_dir_vert(wi, v1, oy1, kDown);
            const unsigned d2_d = out_dir_vert(wj, v2, oy1, kDown);
            if (d1_d && d2_d && d1_d != d2_d) {
              push_joint({mx, oy1});
            }
          }
        }

        // 3. Same-net orthogonal interior crossings.
        auto check_crossings = [&](const WireGeom& hw, const WireGeom& vw) {
          for (const auto& h : hw.h_segs) {
            for (const auto& v : vw.v_segs) {
              if (h.x0 + kEps < v.x && v.x < h.x1 - kEps && v.y0 + kEps < h.y &&
                  h.y < v.y1 - kEps) {
                const Point cross{v.x, h.y};
                if (!is_terminal(*hw.pts, cross) &&
                    !is_terminal(*vw.pts, cross)) {
                  push_joint(cross);
                }
              }
            }
          }
        };
        check_crossings(wi, wj);
        check_crossings(wj, wi);
      }
    }
    g_start = g_end;
  }

  if (raw_joints.empty()) {
    return {};
  }
  std::sort(raw_joints.begin(), raw_joints.end(), [](Point a, Point b) {
    return a.x != b.x ? a.x < b.x : a.y < b.y;
  });
  std::vector<Point> out;
  out.reserve(raw_joints.size());
  for (const Point p : raw_joints) {
    bool dup = false;
    for (std::size_t k = out.size(); k > 0; --k) {
      const Point& prev = out[k - 1];
      if (p.x - prev.x > kEps) {
        break;
      }
      if (std::abs(p.y - prev.y) <= kEps) {
        dup = true;
        break;
      }
    }
    if (!dup) {
      out.push_back(p);
    }
  }
  return out;
}

void LocalOrthogonalRouter::RouteAllParallel(unsigned threads) {
  std::vector<Id> ids;
  ids.reserve(conns_.size());
  for (const auto& entry : conns_) {
    ids.push_back(entry.first);
  }
  if (ids.empty()) {
    return;
  }
  std::sort(ids.begin(), ids.end());
  std::vector<Route> result(ids.size());
  std::atomic<size_t> next{0};
  auto work = [&] {
    for (;;) {
      const size_t i = next.fetch_add(1, std::memory_order_relaxed);
      if (i >= ids.size()) {
        return;
      }
      result[i] = SolveOne(std::as_const(conns_).at(ids[i]), false);
    }
  };
  if (!threads) {
    threads = std::max(1u, std::thread::hardware_concurrency());
  }
  std::vector<std::future<void>> workers;
  for (size_t i = 1; i < std::min<size_t>(threads, ids.size()); ++i) {
    workers.push_back(std::async(std::launch::async, work));
  }
  work();
  for (auto& w : workers) {
    w.get();
  }
  for (size_t i = 0; i < ids.size(); ++i) {
    Publish(ids[i], std::move(result[i]));
  }
  if (cfg_.enable_channel_nudging) {
    (void)ApplyChannelNudging({}, true);
  }
}

void LocalOrthogonalRouter::ExportRegionToSvg(
    const std::string& path, Rect viewport,
    const std::vector<Id>& highlight) const {
  CheckBox(viewport);
  std::ofstream f(path);
  f.exceptions(std::ios::badbit | std::ios::failbit);
  f.imbue(std::locale::classic());
  f.precision(15);
  f << "<svg xmlns='http://www.w3.org/2000/svg' viewBox='" << viewport.l << ' '
    << viewport.t << ' ' << viewport.r - viewport.l << ' '
    << viewport.b - viewport.t << "'>\n";
  for (Id id : index_.Query(viewport)) {
    const auto& b = blocks_.at(id);
    const Rect r = b.rect;
    f << "<rect x='" << r.l << "' y='" << r.t << "' width='" << r.r - r.l
      << "' height='" << r.b - r.t << "' fill='lightgray'/>\n";
    for (const auto& pin : b.pins) {
      const Point p = PinPosition(r, pin);
      f << "<circle cx='" << p.x << "' cy='" << p.y << "' r='3'/>\n";
    }
  }
  const std::set<Id> hi(highlight.begin(), highlight.end());
  for (const auto& entry : routes_) {
    const auto p = entry.second.status == Status::kPending
                       ? Preview(entry.first)
                       : entry.second.points;
    bool visible = false;
    for (size_t i = 1; i < p.size(); ++i) {
      visible |= Box(p[i - 1], p[i]).Intersects(viewport);
    }
    if (!visible) {
      continue;
    }
    f << "<polyline data-id='" << entry.first << "' fill='none' stroke='"
      << (entry.second.status != Status::kValid ? "red"
          : hi.count(entry.first)               ? "blue"
                                                : "#667")
      << (entry.second.status == Status::kPending ? "' stroke-dasharray='4 3"
                                                  : "")
      << "' points='";
    for (const Point pt : p) {
      f << pt.x << ',' << pt.y << ' ';
    }
    f << "'/>\n";
  }
  f << "</svg>\n";
}

}  // namespace localroute
