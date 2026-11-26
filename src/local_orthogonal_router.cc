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
        std::isfinite(cfg.nudge_distance) && cfg.budget >= 2 &&
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
  const Rect aa = src.rect.Pad(cfg_.buffer);
  const Rect bb = dst.rect.Pad(cfg_.buffer);

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
      const Rect rect = blocks_.at(id).rect.Pad(cfg_.buffer);
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

Route LocalOrthogonalRouter::SolveOne(const Conn& c, bool settled) const {
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
  out.points = Elbow(a, b, true);
  out.status = Status::kDisplayOnly;
  Profile* profile = cfg_.profile ? &out.profile : nullptr;
  auto query = [&](Rect rect) {
    Stopwatch clock(profile ? &profile->query_ms : nullptr);
    return index_.Query(rect);
  };
  const Rect aa = src.rect.Pad(cfg_.buffer);
  const Rect bb = dst.rect.Pad(cfg_.buffer);
  // A terminal buried inside another buffered rectangle has no legal exit.
  for (const auto& end : {std::make_pair(a, c.src), std::make_pair(b, c.dst)}) {
    for (Id id : query(Box(end.first, end.first).Pad(cfg_.buffer))) {
      if (id != end.second &&
          blocks_.at(id).rect.Pad(cfg_.buffer).Inside(end.first)) {
        out.outcome = Outcome::kBuriedPin;
        return out;
      }
    }
  }

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
        const Rect rect = blocks_.at(id).rect.Pad(cfg_.buffer);
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

  if (cfg_.fast_paths && !(settled && cfg_.settle_nudge)) {
    const auto it = routes_.find(c.id);
    const bool has_old =
        it != routes_.end() && it->second.method != RouteMethod::kNone;
    // 1. Whole-polyline translation reuse.
    if (has_old) {
      const auto& p = it->second.points;
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
          return out;
        }
      }
    }
    // 2. Clean 2/3-segment L and Z Simple candidates (eliminates stale hooks).
    if (accept(Elbow(a, b, true), RouteMethod::kSimple) ||
        (a.x != b.x && a.y != b.y &&
         accept(Elbow(a, b, false), RouteMethod::kSimple))) {
      return out;
    }
    if (a.y != b.y && (sp.dirs & (kLeft | kRight)) &&
        (dp.dirs & (kLeft | kRight))) {
      for (double x :
           {(a.x + b.x) * 0.5, std::clamp((a.x + b.x) * 0.5, aa.l, aa.r),
            std::clamp((a.x + b.x) * 0.5, bb.l, bb.r)}) {
        if (accept({a, {x, a.y}, {x, b.y}, b}, RouteMethod::kSimple)) {
          return out;
        }
      }
    }
    if (a.x != b.x && (sp.dirs & (kUp | kDown)) && (dp.dirs & (kUp | kDown))) {
      for (double y :
           {(a.y + b.y) * 0.5, std::clamp((a.y + b.y) * 0.5, aa.t, aa.b),
            std::clamp((a.y + b.y) * 0.5, bb.t, bb.b)}) {
        if (accept({a, {a.x, y}, {b.x, y}, b}, RouteMethod::kSimple)) {
          return out;
        }
      }
    }
    // 3. Fallback: patch endpoints onto previous middle segments when Simple is
    //    blocked by obstacles.
    if (has_old) {
      const auto& p = it->second.points;
      if (p.size() >= 3) {
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
          return out;
        }
      }
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
    if (accept(solver_->Solve(
                   {cfg_, c, obs, settled && cfg_.settle_nudge, profile}),
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
    Publish(id, SolveOne(conns_.at(id), settled));
  }
  if (cfg_.enable_channel_nudging && !ids.empty()) {
    (void)ApplyChannelNudging({ids.begin(), ids.end()}, false);
  }
  return {ids.begin(), ids.end()};
}

void LocalOrthogonalRouter::RouteConnector(Id id) {
  Publish(id, SolveOne(conns_.at(id), false));
  if (cfg_.enable_channel_nudging) {
    (void)ApplyChannelNudging({id}, false);
  }
}

void LocalOrthogonalRouter::MarkConnector(Id id) {
  (void)conns_.at(id);
  Mark(id);
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
