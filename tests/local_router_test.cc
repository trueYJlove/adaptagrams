#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "localroute/async_local_router.h"
#include "topology_generator.h"

namespace localroute {
namespace {

#define CHECK(x)                                                           \
  do {                                                                     \
    if (!(x)) {                                                            \
      throw std::runtime_error(#x " at line " + std::to_string(__LINE__)); \
    }                                                                      \
  } while (0)

Block MakeBlock(Id id, Rect r) {
  return {id, r, {{1, 1.0, 0.5, kRight}, {2, 0.0, 0.5, kLeft}}};
}

void AddPair(LocalOrthogonalRouter* r) {
  r->AddBlock(MakeBlock(1, {0, 0, 40, 40}));
  r->AddBlock(MakeBlock(2, {200, 0, 240, 40}));
  r->AddConnector({1, 1, 1, 2, 2});
}

class SpySolver final : public LocalRouteSolver {
 public:
  const char* name() const override { return "spy"; }

  std::vector<Point> Solve(const Request& r) const override {
    seen_.push_back(r.conn.id);
    nudge_ = r.nudge;
    return inner_->Solve(r);
  }

  std::vector<Id>& seen() const { return seen_; }
  bool nudge() const { return nudge_; }

 private:
  std::shared_ptr<const LocalRouteSolver> inner_ = LazyHananAStarSolver();
  mutable std::vector<Id> seen_;
  mutable bool nudge_ = false;
};

void TestGrid() {
  SpatialGridIndex g(100);
  g.Insert(99999, {-30, -30, -20, -20});
  g.Update(99999, {-15, -15, -10, -10});
  CHECK(g.Query({-30, -30, -20, -20}).empty());
  CHECK(g.Query({-10, -10, -10, -10}) == std::vector<Id>{99999});
  g.Remove(99999);
  std::mt19937 rng(42);
  std::vector<Rect> v;
  for (Id i = 0; i < 10000; ++i) {
    const double x = static_cast<int>(rng() % 8000) - 4000;
    const double y = static_cast<int>(rng() % 8000) - 4000;
    v.push_back({x, y, x + 50, y + 60});
    g.Insert(i, v.back());
  }
  for (Id i = 0; i < 10000; i += 2) {
    v[i] = v[i].Pad(40);
    g.Update(i, v[i]);
  }
  for (Id i = 0; i < 10000; i += 3) {
    g.Remove(i);
  }
  for (int j = 0; j < 100; ++j) {
    const double x = static_cast<int>(rng() % 8000) - 4000;
    const double y = static_cast<int>(rng() % 8000) - 4000;
    const Rect q{x, y, x + 350, y + 400};
    std::vector<Id> want;
    for (Id i = 0; i < 10000; ++i) {
      if (i % 3 && v[i].Intersects(q)) {
        want.push_back(i);
      }
    }
    CHECK(g.Query(q) == want);
  }
}

void TestDirty() {
  auto s = std::make_shared<SpySolver>();
  Config cfg;
  CHECK(!cfg.reroute_third_party_blocked);
  cfg.settle_nudge = true;
  cfg.fast_paths = false;
  LocalOrthogonalRouter r(cfg, s);
  AddPair(&r);
  r.AddBlock(MakeBlock(3, {100, 80, 140, 120}));
  r.AddBlock(MakeBlock(4, {300, 80, 340, 120}));
  r.AddConnector({2, 3, 1, 4, 2});
  r.RouteAllParallel(1);
  const auto old1 = r.route(1).points;
  CHECK(r.route(1).status == Status::kValid && old1.size() >= 2);
  s->seen().clear();
  r.MoveBlocksBatch({{3, {100, 5, 140, 45}}});
  CHECK(r.route(1).status == Status::kValid);
  CHECK(r.route(2).status == Status::kPending);
  CHECK(r.RerouteAffected({3, 3}) == std::vector<Id>{2});
  CHECK(s->seen() == std::vector<Id>{2});
  CHECK(r.route(1).points == old1);
  s->seen().clear();
  r.RerouteAffected({3}, true);
  CHECK(s->nudge() && s->seen() == std::vector<Id>{2});
  r.RemoveBlock(3);
  r.MoveBlock(4, 0, 5);
  CHECK(r.RerouteAffected({4}).empty());
}

void TestAvoid() {
  Config cfg;
  cfg.fast_paths = false;
  LocalOrthogonalRouter r(cfg);
  AddPair(&r);
  r.AddBlock(MakeBlock(3, {100, -10, 140, 50}));
  r.AddBlock(MakeBlock(4, {220, 120, 260, 160}));
  r.AddConnector({2, 1, 1, 4, 2});
  r.RouteAllParallel(1);
  for (Id id : {1u, 2u}) {
    const auto a = r.route(id);
    CHECK(a.status == Status::kValid);
    CHECK(a.points.size() >= 4);
    CHECK(a.points[1].x > a.points[0].x);
    CHECK(a.points[a.points.size() - 2].x < a.points.back().x);
    for (size_t j = 1; j < a.points.size(); ++j) {
      const Rect seg = Box(a.points[j - 1], a.points[j]);
      CHECK(seg.l == seg.r || seg.t == seg.b);
      CHECK(!(seg.l < 148 && seg.r > 92 && seg.t < 58 && seg.b > -18));
    }
  }
}

class WallSolver final : public LocalRouteSolver {
 public:
  const char* name() const override { return "wall-test"; }
  std::vector<Point> Solve(const Request& q) const override {
    for (const Block* o : q.obstacles) {
      if (o->id == 9) {
        return {{40, 20},   {45, 20},  {45, -80},
                {195, -80}, {195, 20}, {200, 20}};
      }
    }
    return {{40, 20}, {200, 20}};
  }
};

void TestExpansion() {
  Config cfg;
  cfg.buffer = 4;
  cfg.padding = 10;
  cfg.fast_paths = false;
  cfg.budget = 3;
  auto s = std::make_shared<WallSolver>();
  LocalOrthogonalRouter r(cfg, s);
  AddPair(&r);
  r.AddBlock(MakeBlock(8, {75, 26, 85, 34}));
  r.AddBlock(MakeBlock(9, {110, -65, 125, 35}));
  r.RouteAllParallel(1);
  const auto a = r.route(1);
  CHECK(a.status == Status::kValid);
  CHECK(a.retries == 1 && a.calls == 2 && a.window == 3);
  cfg.budget = 2;
  LocalOrthogonalRouter limited(cfg, s);
  AddPair(&limited);
  limited.AddBlock(MakeBlock(9, {110, -65, 125, 35}));
  limited.RouteAllParallel(1);
  CHECK(limited.route(1).status == Status::kDisplayOnly);
}

void TestOverlap() {
  LocalOrthogonalRouter r;
  AddPair(&r);
  r.MoveBlocksBatch({{2, {0, 0, 40, 40}}});
  r.RouteAllParallel(1);
  CHECK(r.route(1).status != Status::kValid);
  r.MoveBlocksBatch({{2, {40, 0, 80, 40}}});
  r.RouteAllParallel(1);
  CHECK(r.route(1).status == Status::kFailed);
  r.MoveBlocksBatch({{2, {200, 0, 240, 40}}});
  r.AddBlock(MakeBlock(3, {30, 10, 65, 30}));
  r.RouteAllParallel(1);
  CHECK(r.route(1).status == Status::kDisplayOnly && r.route(1).calls == 0);

  // Close forward flush Right -> Left aligned blocks: 2-point short direct.
  {
    LocalOrthogonalRouter flush;
    flush.AddBlock(MakeBlock(1, {0.0, 0.0, 40.0, 40.0}));
    flush.AddBlock(MakeBlock(2, {44.0, 0.0, 84.0, 40.0}));
    flush.AddConnector({1, 1, 1, 2, 2});
    flush.RouteAllParallel(1);
    const Route& rt = flush.route(1);
    CHECK(rt.status == Status::kDisplayOnly);
    CHECK(rt.points.size() == 2);
    CHECK((rt.points[0] == Point{40.0, 20.0}));
    CHECK((rt.points[1] == Point{44.0, 20.0}));
  }

  // Overlapping Right -> Left reversed blocks: Simulink outer-loop fallback.
  {
    LocalOrthogonalRouter ov;
    const Rect r1{100.0, 0.0, 180.0, 60.0};
    const Rect r2{110.0, 10.0, 190.0, 70.0};
    ov.AddBlock(MakeBlock(1, r1));
    ov.AddBlock(MakeBlock(2, r2));
    ov.AddConnector({1, 1, 1, 2, 2});
    ov.RouteAllParallel(1);
    const Route& rt = ov.route(1);
    CHECK(rt.status == Status::kDisplayOnly);
    CHECK(rt.points.size() == 6);
    // First leg exits strictly right, last leg enters from left going right.
    CHECK(rt.points[0].y == rt.points[1].y);
    CHECK(rt.points[1].x > rt.points[0].x);
    CHECK(rt.points[4].y == rt.points[5].y);
    CHECK(rt.points[4].x < rt.points[5].x);
    // Horizontal cross-rail lies strictly outside the union bounding box U.
    const double u_top = std::min(r1.t, r2.t);
    const double u_bot = std::max(r1.b, r2.b);
    CHECK(rt.points[2].y == rt.points[3].y);
    CHECK(rt.points[2].y < u_top || rt.points[2].y > u_bot);
  }

  // Non-overlapping far blocks with a buried pin must NOT trigger a full-span
  // outer loop; must remain a compact L/Z inside Box(a, b).
  {
    LocalOrthogonalRouter sep;
    sep.AddBlock(MakeBlock(1, {0.0, 0.0, 80.0, 50.0}));
    sep.AddBlock(MakeBlock(2, {400.0, 120.0, 480.0, 170.0}));
    sep.AddBlock(MakeBlock(3, {70.0, 0.0, 110.0, 50.0}));  // Buries pin (80,25)
    sep.AddConnector({1, 1, 1, 2, 2});
    sep.RouteAllParallel(1);
    const Route& rt = sep.route(1);
    CHECK(rt.status == Status::kDisplayOnly);
    CHECK(rt.outcome == Outcome::kBuriedPin);
    CHECK(rt.points.size() <= 4);
    for (const Point p : rt.points) {
      CHECK(p.x >= 80.0 && p.x <= 400.0);
      CHECK(p.y >= 25.0 && p.y <= 145.0);
    }
  }
}

void TestParallel() {
  LocalOrthogonalRouter r;
  for (Id i = 0; i < 500; ++i) {
    const double x = (i % 25) * 160.0;
    const double y = (i / 25) * 90.0;
    r.AddBlock(MakeBlock(i, {x, y, x + 70, y + 40}));
    if (i % 25) {
      r.AddConnector({i, i - 1, 1, i, 2});
    }
  }
  r.RouteAllParallel(1);
  std::vector<Route> ref(500);
  for (Id i = 1; i < 500; ++i) {
    if (i % 25) {
      ref[i] = r.route(i);
      CHECK(ref[i].status == Status::kValid);
    }
  }
  for (int rep = 0; rep < 4; ++rep) {
    r.RouteAllParallel(4);
    for (Id i = 1; i < 500; ++i) {
      if (i % 25) {
        const auto& q = r.route(i);
        CHECK(q.status == ref[i].status && q.points == ref[i].points);
      }
    }
  }
}

void TestFastAndObstruction() {
  auto s = std::make_shared<SpySolver>();
  Config cfg;
  cfg.reroute_third_party_blocked = true;
  LocalOrthogonalRouter r(cfg, s);
  AddPair(&r);
  r.AddBlock(MakeBlock(3, {100, 80, 140, 120}));
  r.RouteAllParallel(1);
  CHECK(r.route(1).status == Status::kValid &&
        r.route(1).method == RouteMethod::kSimple);
  r.MoveBlocksBatch({{1, {0, 5, 40, 45}}, {2, {200, 5, 240, 45}}});
  CHECK(r.route(1).status == Status::kPending);
  CHECK(r.RerouteAffected({1, 2}) == std::vector<Id>{1});
  CHECK(r.route(1).method == RouteMethod::kReuse && s->seen().empty());
  r.MoveBlock(3, 0, -80);
  CHECK(r.route(1).status == Status::kPending);
  CHECK(r.RerouteAffected({3}) == std::vector<Id>{1});
  CHECK(r.route(1).status == Status::kValid &&
        r.route(1).method == RouteMethod::kNative);
  CHECK(s->seen() == std::vector<Id>{1});
  r.MoveBlock(1, 0, 2);
  CHECK(r.RerouteAffected({1}) == std::vector<Id>{1});
  CHECK(r.route(1).status == Status::kValid &&
        r.route(1).method == RouteMethod::kReuse);
  CHECK(s->seen() == std::vector<Id>{1});
  r.RemoveConnector(1);
  CHECK(r.RerouteAffected({3}).empty());
}

class PausedSolver final : public LocalRouteSolver {
 public:
  explicit PausedSolver(std::shared_future<void> release)
      : release_(std::move(release)) {}

  const char* name() const override { return "paused"; }

  std::vector<Point> Solve(const Request& q) const override {
    if (++calls_ == 1) {
      entered_.set_value();
      release_.wait();
    }
    return inner_->Solve(q);
  }

  std::future<void> GetEnteredFuture() { return entered_.get_future(); }

 private:
  std::shared_ptr<const LocalRouteSolver> inner_ = LazyHananAStarSolver();
  mutable std::promise<void> entered_;
  std::shared_future<void> release_;
  mutable unsigned calls_ = 0;
};

void TestAsyncRouting() {
  using namespace std::chrono_literals;
  std::promise<void> release;
  auto s = std::make_shared<PausedSolver>(release.get_future().share());
  auto entered = s->GetEnteredFuture();
  Config cfg;
  cfg.fast_paths = false;
  LocalOrthogonalRouter r(cfg, s);
  AddPair(&r);
  AsyncLocalRouter router(std::move(r));
  router.StartAll();
  const auto first = entered.wait_for(5s);
  const auto pre = router.view().route(1).status;
  router.Move({{1, {0, 4, 40, 44}}});
  router.Move({{1, {0, 16, 40, 56}}});
  const auto before = router.Poll();
  const auto ghost = router.view().Preview(1).front();
  router.Settle();
  release.set_value();
  CHECK(first == std::future_status::ready && pre == Status::kPending &&
        before == 0 && (ghost == Point{40, 36}));
  const auto limit = std::chrono::steady_clock::now() + 5s;
  while (router.PendingCount() && std::chrono::steady_clock::now() < limit) {
    router.Poll();
    std::this_thread::sleep_for(1ms);
  }
  const auto& out = router.view().route(1);
  CHECK(router.PendingCount() == 0 && out.status == Status::kValid &&
        (out.points.front() == Point{40, 36}));
}

void TestBoundaryCases() {
  Config cfg;
  cfg.fast_paths = false;
  cfg.buffer = 8;

  {
    LocalOrthogonalRouter r(cfg);
    r.AddBlock(MakeBlock(1, {0, 0, 40, 40}));
    r.AddBlock(MakeBlock(2, {200, 0, 240, 40}));
    r.AddBlock(MakeBlock(3, {80, -20, 120, 60}));
    r.AddBlock(MakeBlock(4, {60, -100, 140, -36}));
    r.AddConnector({1, 1, 1, 2, 2});

    r.AddBlock(MakeBlock(10, {0, 200, 40, 240}));
    r.AddBlock(MakeBlock(11, {240, 320, 280, 360}));
    r.AddBlock(MakeBlock(12, {90, 120, 220, 260}));
    r.AddBlock(MakeBlock(13, {0, 320, 180, 380}));
    r.AddConnector({2, 10, 1, 11, 2});

    r.RouteAllParallel(1);
    CHECK(r.route(1).status == Status::kValid);
    CHECK(r.route(2).status == Status::kValid);
    bool passed_corridor_center = false;
    for (const auto& pt : r.route(2).points) {
      if (std::abs(pt.y - 290.0) < 1.0) {
        passed_corridor_center = true;
      }
    }
    CHECK(passed_corridor_center);
  }

  {
    LocalOrthogonalRouter r(cfg);
    r.AddBlock({1, {0, 100, 40, 140}, {{1, 1, 0.5, kRight}}});
    for (Id i = 0; i < 4; ++i) {
      const double y = 20.0 + i * 70.0;
      r.AddBlock(MakeBlock(10 + i, {220, y, 260, y + 40}));
      r.AddConnector({100 + i, 1, 1, 10 + i, 2});
    }
    r.AddBlock(MakeBlock(50, {110, 80, 150, 160}));
    r.RouteAllParallel(1);
    for (Id i = 0; i < 4; ++i) {
      CHECK(r.route(100 + i).status == Status::kValid);
    }
  }

  {
    LocalOrthogonalRouter r(cfg);
    r.AddBlock(MakeBlock(1, {0, 0, 40, 40}));
    r.AddBlock(MakeBlock(2, {260, 0, 300, 40}));
    r.AddBlock(MakeBlock(3, {70, -60, 100, 60}));
    r.AddBlock(MakeBlock(4, {-20, -60, 65, -25}));
    r.AddBlock(MakeBlock(5, {200, -10, 230, 100}));
    r.AddBlock(MakeBlock(6, {235, 65, 320, 100}));
    r.AddConnector({1, 1, 1, 2, 2});
    r.RouteAllParallel(1);
    CHECK(r.route(1).status == Status::kValid);
  }
}

bool SegmentsProperlyCross(Point a0, Point a1, Point b0, Point b1) {
  const bool a_vert = (a0.x == a1.x);
  const bool b_vert = (b0.x == b1.x);
  if (a_vert == b_vert) {
    return false;
  }
  const Point h0 = a_vert ? b0 : a0;
  const Point h1 = a_vert ? b1 : a1;
  const Point v0 = a_vert ? a0 : b0;
  const Point v1 = a_vert ? a1 : b1;
  const double hx_min = std::min(h0.x, h1.x);
  const double hx_max = std::max(h0.x, h1.x);
  const double vy_min = std::min(v0.y, v1.y);
  const double vy_max = std::max(v0.y, v1.y);
  return v0.x > hx_min + 1e-6 && v0.x < hx_max - 1e-6 && h0.y > vy_min + 1e-6 &&
         h0.y < vy_max - 1e-6;
}

void TestChannelNudgingParallelBundle() {
  Config cfg;
  cfg.nudge_distance = 6.0;
  cfg.enable_channel_nudging = true;
  LocalOrthogonalRouter r(cfg);

  std::vector<Pin> b1_pins;
  std::vector<Pin> b2_pins;
  for (Id i = 0; i < 10; ++i) {
    const double y = static_cast<double>(i + 1) / 11.0;
    b1_pins.push_back({100 + i, 1.0, y, kRight});
    b2_pins.push_back({200 + i, 0.0, y, kLeft});
  }
  r.AddBlock({1, {0.0, 0.0, 80.0, 260.0}, b1_pins});
  r.AddBlock({2, {340.0, 60.0, 420.0, 320.0}, b2_pins});
  for (Id i = 0; i < 10; ++i) {
    r.AddConnector({100 + i, 1, 100 + i, 2, 200 + i});
  }

  r.RouteAllParallel(1);

  std::vector<double> mid_xs;
  for (Id i = 0; i < 10; ++i) {
    const Route& rt = r.route(100 + i);
    CHECK(rt.status == Status::kValid);
    CHECK(rt.points.size() == 4);
    mid_xs.push_back(rt.points[1].x);
  }
  std::sort(mid_xs.begin(), mid_xs.end());
  for (std::size_t i = 1; i < mid_xs.size(); ++i) {
    CHECK(mid_xs[i] - mid_xs[i - 1] >= 4.0);
  }
  for (Id i = 0; i < 10; ++i) {
    const auto& pa = r.route(100 + i).points;
    for (Id j = i + 1; j < 10; ++j) {
      const auto& pb = r.route(100 + j).points;
      for (std::size_t sa = 1; sa < pa.size(); ++sa) {
        for (std::size_t sb = 1; sb < pb.size(); ++sb) {
          CHECK(!SegmentsProperlyCross(pa[sa - 1], pa[sa], pb[sb - 1], pb[sb]));
        }
      }
    }
  }
}

void TestChannelNudgingCascadeAndNoHook() {
  Config cfg;
  cfg.nudge_distance = 6.0;
  cfg.enable_channel_nudging = true;
  LocalOrthogonalRouter r(cfg);

  // Start B9 to the right of B10 so its initial route wraps around with >= 4
  // bends (6+ points), then drag B9 back to the left column to ensure Simple
  // snaps the hook away immediately.
  r.AddBlock({9, {300.0, 0.0, 390.0, 50.0}, {{1, 1.0, 0.5, kRight}}});
  r.AddBlock({3, {0.0, 80.0, 90.0, 130.0}, {{1, 1.0, 0.5, kRight}}});
  r.AddBlock({8, {0.0, 160.0, 90.0, 210.0}, {{1, 1.0, 0.5, kRight}}});
  r.AddBlock({10,
              {270.0, 150.0, 360.0, 230.0},
              {{1, 0.0, 0.33, kLeft}, {2, 0.0, 0.67, kLeft}}});
  r.AddBlock({11, {270.0, 240.0, 360.0, 290.0}, {{1, 0.0, 0.5, kLeft}}});
  r.AddBlock({12, {270.0, 320.0, 360.0, 370.0}, {{1, 0.0, 0.5, kLeft}}});

  r.AddConnector({1, 9, 1, 10, 1});
  r.AddConnector({2, 3, 1, 10, 2});
  r.AddConnector({3, 8, 1, 11, 1});
  r.AddConnector({4, 8, 1, 12, 1});  // Shares NetKey(8, 1) with conn 3.

  r.RouteAllParallel(1);
  CHECK(r.route(1).status == Status::kValid);
  CHECK(r.route(1).points.size() >= 6);  // Wrapped around from right side.
  const std::vector<Point> frozen_b3_initial = r.route(2).points;
  const std::vector<Point> frozen_b8_initial = r.route(3).points;

  // Drag B9 left into the source column: Simple Z must eliminate the stale
  // hook (<= 4 points) and ChannelNudger must separate B9 without moving
  // non-adjacent routes B3/B8.
  r.MoveBlock(9, -300.0, 0.0);
  CHECK(r.RerouteAffected({9}) == std::vector<Id>{1});
  CHECK(r.route(2).points == frozen_b3_initial);
  CHECK(r.route(3).points == frozen_b8_initial);

  for (Id cid : {1u, 2u, 3u, 4u}) {
    const Route& rt = r.route(cid);
    CHECK(rt.status == Status::kValid);
    CHECK(rt.points.size() <= 4);  // At most 2 bends (no 4-bend hook).
  }

  // Same-source fan-out (3 and 4) must keep sharing their middle trunk.
  CHECK(std::abs(r.route(3).points[1].x - r.route(4).points[1].x) < 1e-6);

  // Three distinct nets (B9, B3, B8) must be separated >= 4px without crossing.
  std::vector<double> xs{r.route(1).points[1].x, r.route(2).points[1].x,
                         r.route(3).points[1].x};
  std::sort(xs.begin(), xs.end());
  CHECK(xs[1] - xs[0] >= 4.0);
  CHECK(xs[2] - xs[1] >= 4.0);

  for (Id a : {1u, 2u, 3u}) {
    const auto& pa = r.route(a).points;
    for (Id b : {1u, 2u, 3u}) {
      if (a >= b) {
        continue;
      }
      const auto& pb = r.route(b).points;
      for (std::size_t sa = 1; sa < pa.size(); ++sa) {
        for (std::size_t sb = 1; sb < pb.size(); ++sb) {
          CHECK(!SegmentsProperlyCross(pa[sa - 1], pa[sa], pb[sb - 1], pb[sb]));
        }
      }
    }
  }

  // Drag B3 continuously rightward near/across B9->B10's vertical segment:
  // B9->B10 and B8->B11 must remain bit-for-bit stationary while B3->B10
  // keeps >= 4px separation from overlapping vertical segments.
  const std::vector<Point> frozen_b9 = r.route(1).points;
  const std::vector<Point> frozen_b8 = r.route(3).points;
  const double b9_mid_x = frozen_b9[1].x;
  const double b9_min_y = std::min(frozen_b9[1].y, frozen_b9[2].y);
  const double b9_max_y = std::max(frozen_b9[1].y, frozen_b9[2].y);
  for (int step = 0; step < 12; ++step) {
    r.MoveBlock(3, 10.0, 0.0);
    CHECK(r.RerouteAffected({3}) == std::vector<Id>{2});
    CHECK(r.route(1).points == frozen_b9);
    CHECK(r.route(3).points == frozen_b8);
    const Route& b3_rt = r.route(2);
    CHECK(b3_rt.status == Status::kValid);
    for (std::size_t j = 1; j < b3_rt.points.size(); ++j) {
      const Point u = b3_rt.points[j - 1];
      const Point v = b3_rt.points[j];
      if (std::abs(u.x - v.x) <= 1e-6) {
        const double ymin = std::min(u.y, v.y);
        const double ymax = std::max(u.y, v.y);
        if (ymin < b9_max_y - 1e-3 && b9_min_y < ymax - 1e-3) {
          CHECK(std::abs(u.x - b9_mid_x) >= 4.0);
        }
      }
    }
  }
}

void TestNarrowGapAvoidance() {
  Config cfg;
  cfg.buffer = 8.0;
  cfg.fast_paths = false;
  LocalOrthogonalRouter r(cfg);

  // B9 and B8 leave only a 5px narrow gap (< buffer = 8px) right next to B9's
  // output pin, while B3 blocks the upper-right corridor; B9 -> B10 must shrink
  // the nearby buffer dynamically, escape cleanly without body intersection or
  // full-screen outer loop, and route Status::kValid around B8 and B3.
  const Rect b9_rect{0.0, 0.0, 90.0, 40.0};
  const Rect b8_rect{95.0, 0.0, 175.0, 90.0};  // 5px horizontal gap from B9
  const Rect b3_rect{0.0, 45.0, 90.0, 95.0};   // 5px vertical gap below B9
  const Rect b10_rect{240.0, 20.0, 320.0, 80.0};

  r.AddBlock({9, b9_rect, {{1, 1.0, 0.5, kRight}}});
  r.AddBlock({8, b8_rect, {}});
  r.AddBlock({3, b3_rect, {}});
  r.AddBlock({10, b10_rect, {{1, 0.0, 0.5, kLeft}}});
  r.AddConnector({1, 9, 1, 10, 1});

  r.RouteAllParallel(1);
  const Route& rt = r.route(1);
  CHECK(rt.status == Status::kValid);
  CHECK(rt.points.size() >= 4);
  CHECK(rt.points[1].x > rt.points[0].x && rt.points[1].x < b8_rect.l);
  for (std::size_t j = 1; j < rt.points.size(); ++j) {
    const Rect seg = Box(rt.points[j - 1], rt.points[j]);
    CHECK(seg.l == seg.r || seg.t == seg.b);
    CHECK(!(seg.r > b8_rect.l && seg.l < b8_rect.r && seg.b > b8_rect.t &&
            seg.t < b8_rect.b));
    CHECK(!(seg.r > b3_rect.l && seg.l < b3_rect.r && seg.b > b3_rect.t &&
            seg.t < b3_rect.b));
  }
}

void TestSolderJointDetection() {
  Config cfg;
  cfg.enable_channel_nudging = true;
  LocalOrthogonalRouter r(cfg);

  // Block 1 (right pin at (80, 70)) fans out to Blocks 2, 3, and 4.
  r.AddBlock({1, {0.0, 40.0, 80.0, 100.0}, {{1, 1.0, 0.5, kRight}}});
  r.AddBlock({2, {280.0, -40.0, 360.0, 20.0}, {{2, 0.0, 0.5, kLeft}}});
  r.AddBlock({3, {280.0, 40.0, 360.0, 100.0}, {{2, 0.0, 0.5, kLeft}}});
  r.AddBlock({4, {280.0, 120.0, 360.0, 180.0}, {{2, 0.0, 0.5, kLeft}}});

  // Unrelated crossing net: Block 5 -> Block 6 vertical at x = 120 crosses y
  // = 70.
  r.AddBlock({5, {95.0, -100.0, 145.0, -50.0}, {{3, 0.5, 1.0, kDown}}});
  r.AddBlock({6, {95.0, 200.0, 145.0, 250.0}, {{4, 0.5, 0.0, kUp}}});

  r.AddConnector({1, 1, 1, 2, 2});
  r.AddConnector({2, 1, 1, 3, 2});
  r.AddConnector({3, 1, 1, 4, 2});
  r.AddConnector({4, 5, 3, 6, 4});

  r.RouteAllParallel(1);
  for (Id cid : {1u, 2u, 3u, 4u}) {
    CHECK(r.route(cid).status == Status::kValid);
  }

  // All 3 fan-out branches plus crossing net -> exactly 1 solder joint at
  // (180, 70); the different-net orthogonal crossing at (120, 70) must NOT
  // produce a solder joint.
  const std::vector<Point> joints_all = r.ComputeSolderJoints({1, 2, 3, 4});
  CHECK(joints_all.size() == 1);
  CHECK(std::abs(joints_all[0].x - 180.0) <= 1e-3);
  CHECK(std::abs(joints_all[0].y - 70.0) <= 1e-3);

  // Even without the straight middle wire, shared trunk divergence of 1 & 3
  // still detects the solder joint at (180, 70).
  const std::vector<Point> joints_split = r.ComputeSolderJoints({1, 3});
  CHECK(joints_split.size() == 1);
  CHECK(std::abs(joints_split[0].x - 180.0) <= 1e-3);
  CHECK(std::abs(joints_split[0].y - 70.0) <= 1e-3);

  // Different-net crossing alone produces zero solder joints.
  CHECK(r.ComputeSolderJoints({2, 4}).empty());
}

void TestManualSegmentDragAndPreserve() {
  Config cfg;
  cfg.nudge_distance = 6.0;
  cfg.enable_channel_nudging = true;
  LocalOrthogonalRouter r(cfg);

  // Conn 1: Z-route from (80, 30) to (280, 110), default vertical mid at x=180.
  r.AddBlock({1, {0.0, 0.0, 80.0, 60.0}, {{1, 1.0, 0.5, kRight}}});
  r.AddBlock({2, {280.0, 80.0, 360.0, 140.0}, {{2, 0.0, 0.5, kLeft}}});
  r.AddConnector({1, 1, 1, 2, 2});

  // Conn 2: 2-point horizontal wire (80, 230) -> (280, 230).
  r.AddBlock({3, {0.0, 200.0, 80.0, 260.0}, {{1, 1.0, 0.5, kRight}}});
  r.AddBlock({4, {280.0, 200.0, 360.0, 260.0}, {{2, 0.0, 0.5, kLeft}}});
  r.AddConnector({2, 3, 1, 4, 2});

  // Conn 3: independent wire whose midpoint can overlap x = 210 to test
  // nudging.
  r.AddBlock({5, {40.0, 70.0, 120.0, 110.0}, {{1, 1.0, 0.5, kRight}}});
  r.AddBlock({6, {280.0, 0.0, 360.0, 40.0}, {{2, 0.0, 0.5, kLeft}}});
  r.AddConnector({3, 5, 1, 6, 2});

  r.RouteAllParallel(1);
  CHECK(r.route(1).status == Status::kValid && !r.route(1).user_modified);
  CHECK(r.route(2).status == Status::kValid && r.route(2).points.size() == 2);

  // Drag conn 1's middle vertical segment (seg 1) to x = 210.
  CHECK(r.MoveRouteSegment(1, 1, 210.0));
  CHECK(r.route(1).status == Status::kValid);
  CHECK(r.route(1).user_modified);
  CHECK(r.route(1).method == RouteMethod::kReuse);
  CHECK(std::abs(r.route(1).points[1].x - 210.0) <= 1e-6);
  CHECK(std::abs(r.route(1).points[2].x - 210.0) <= 1e-6);

  // Move Block 1 slightly (even on release settled=true): hand-adjusted middle
  // segment at x = 210 must be preserved ahead of Simple L/Z.
  r.MoveBlock(1, 0.0, 10.0);
  r.RerouteAffected({1}, true);
  CHECK(r.route(1).status == Status::kValid);
  CHECK(r.route(1).user_modified);
  CHECK(std::abs(r.route(1).points[1].x - 210.0) <= 1e-6);
  CHECK(std::abs(r.route(1).points[2].x - 210.0) <= 1e-6);

  // Move Block 5 so conn 3's midpoint lands at x = 210: ChannelNudger must
  // treat conn 1 as a fixed anchor and move conn 3 away, leaving conn 1 at x =
  // 210.
  r.MoveBlock(5, 20.0, 0.0);  // B5 right becomes 140, mid(140, 280) = 210.
  r.RerouteAffected({5}, false);
  CHECK(std::abs(r.route(1).points[1].x - 210.0) <= 1e-6);
  CHECK(r.route(3).status == Status::kValid);
  CHECK(std::abs(r.route(3).points[1].x - 210.0) >= 4.0);

  // Drag 2-point direct conn 2 vertically to y = 175: inserts orthogonal Z/U.
  CHECK(r.MoveRouteSegment(2, 0, 175.0));
  CHECK(r.route(2).status == Status::kValid);
  CHECK(r.route(2).user_modified);
  CHECK(r.route(2).points.size() >= 4);
  bool has_horiz_175 = false;
  for (std::size_t j = 1; j < r.route(2).points.size(); ++j) {
    if (std::abs(r.route(2).points[j - 1].y - 175.0) <= 1e-6 &&
        std::abs(r.route(2).points[j].y - 175.0) <= 1e-6) {
      has_horiz_175 = true;
    }
  }
  CHECK(has_horiz_175);

  // Auto-route conn 2 clears user_modified and restores straight 2-point line.
  r.AutoRouteConnectors({2});
  CHECK(!r.route(2).user_modified);
  CHECK(r.route(2).status == Status::kValid);
  CHECK(r.route(2).points.size() == 2);

  // If an obstacle blocks the locked segment x = 210 and Block 1 moves,
  // fallback to normal routing clears user_modified and avoids the obstacle.
  r.AddBlock({9, {195.0, 20.0, 225.0, 130.0}, {}});
  r.MoveBlock(1, 0.0, -5.0);
  r.RerouteAffected({1}, false);
  CHECK(r.route(1).status == Status::kValid);
  CHECK(!r.route(1).user_modified);
}

void TestAutoRouteSelectedLines() {
  Config cfg;
  cfg.nudge_distance = 6.0;
  cfg.enable_channel_nudging = true;
  LocalOrthogonalRouter r(cfg);

  r.AddBlock({1, {0.0, 0.0, 80.0, 50.0}, {{1, 1.0, 0.5, kRight}}});
  r.AddBlock({2, {280.0, 80.0, 360.0, 130.0}, {{2, 0.0, 0.5, kLeft}}});
  r.AddBlock({3, {0.0, 70.0, 80.0, 120.0}, {{1, 1.0, 0.5, kRight}}});
  r.AddBlock({4, {280.0, 140.0, 360.0, 190.0}, {{2, 0.0, 0.5, kLeft}}});
  // Obstacle in the middle corridor (x in [150, 210], y in [-10, 200]).
  const Rect obs{150.0, -10.0, 210.0, 200.0};
  r.AddBlock({5, obs, {}});

  r.AddConnector({1, 1, 1, 2, 2});
  r.AddConnector({2, 3, 1, 4, 2});
  r.RouteAllParallel(1);

  // Force both wires through the obstacle via manual segment drag ->
  // DisplayOnly.
  CHECK(r.MoveRouteSegment(1, 1, 180.0));
  CHECK(r.MoveRouteSegment(2, 1, 180.0));
  CHECK(r.route(1).user_modified && r.route(1).status == Status::kDisplayOnly);
  CHECK(r.route(2).user_modified && r.route(2).status == Status::kDisplayOnly);

  // Auto-route selected wires via AsyncLocalRouter to also verify async parity.
  AsyncLocalRouter async_router(std::move(r));
  const std::vector<Id> updated = async_router.AutoRouteConnectors({1, 2});
  CHECK(updated.size() >= 2);
  for (Id cid : {1u, 2u}) {
    const Route& rt = async_router.view().route(cid);
    CHECK(!rt.user_modified);
    CHECK(rt.status == Status::kValid);
    for (std::size_t j = 1; j < rt.points.size(); ++j) {
      const Rect seg = Box(rt.points[j - 1], rt.points[j]);
      CHECK(
          !(seg.r > obs.l && seg.l < obs.r && seg.b > obs.t && seg.t < obs.b));
    }
  }
}

void TestWireCongestionSoftPenalty() {
  Config cfg;
  cfg.buffer = 5.0;
  cfg.penalty = 50.0;
  cfg.wire_overlap_penalty = 25.0;
  auto solver = LazyHananAStarSolver();

  // Symmetric dual narrow-slit corridor layout:
  // src pin at (60, 120), dst pin at (300, 120); center obstacle blocks y=120.
  // Upper narrow slit: b_upper.b + buf = 105, b_center.t - buf = 105 -> track
  // at y = 105 (detour |105 - 120| = 15, total +30).
  // Lower narrow slit: b_center.b + buf = 138, b_lower.t - buf = 138 -> track
  // at y = 138 (detour |138 - 120| = 18, total +36).
  const Block b_src{1, {0.0, 90.0, 60.0, 150.0}, {{1, 1.0, 0.5, kRight}}};
  const Block b_dst{2, {300.0, 90.0, 360.0, 150.0}, {{2, 0.0, 0.5, kLeft}}};
  const Block b_upper{10, {110.0, 40.0, 230.0, 100.0}, {}};
  const Block b_center{11, {110.0, 110.0, 230.0, 133.0}, {}};
  const Block b_lower{12, {110.0, 143.0, 230.0, 200.0}, {}};
  const std::vector<const Block*> obs{&b_src, &b_dst, &b_upper, &b_center,
                                      &b_lower};
  const Conn conn{1, 1, 1, 2, 2};

  auto mean_mid_y = [](const std::vector<Point>& pts) {
    for (std::size_t j = 1; j < pts.size(); ++j) {
      if (std::abs(pts[j - 1].y - pts[j].y) <= 1e-6 &&
          std::min(pts[j - 1].x, pts[j].x) <= 120.0 &&
          std::max(pts[j - 1].x, pts[j].x) >= 220.0) {
        return pts[j].y;
      }
    }
    return 120.0;
  };

  // 1. Baseline (no occupied wires): picks slightly shorter upper corridor.
  const std::vector<Point> base_pts =
      solver->Solve({cfg, conn, obs, false, nullptr, nullptr});
  CHECK(base_pts.size() >= 4);
  CHECK(mean_mid_y(base_pts) < 110.0);

  // 2. Different-source wire occupies upper corridor (y = 105): penalty (25) >
  //    extra detour (6), so solver switches to free lower corridor (y > 135).
  const std::vector<OccupiedSegment> other_net_occ{
      {{90.0, 105.0}, {250.0, 105.0}, 99, 1}};
  const std::vector<Point> detour_pts =
      solver->Solve({cfg, conn, obs, false, nullptr, &other_net_occ});
  CHECK(detour_pts.size() >= 4);
  CHECK(mean_mid_y(detour_pts) > 135.0);

  // 3. Same-source fan-out wire (NetKey {1, 1}) occupies upper corridor: NO
  //    penalty is charged, so solver continues sharing the upper corridor.
  const std::vector<OccupiedSegment> same_net_occ{
      {{90.0, 105.0}, {250.0, 105.0}, 1, 1}};
  const std::vector<Point> shared_pts =
      solver->Solve({cfg, conn, obs, false, nullptr, &same_net_occ});
  CHECK(shared_pts.size() >= 4);
  CHECK(mean_mid_y(shared_pts) < 110.0);
}

void TestUpdateBlockPinsPreservesValidWires() {
  Config cfg;
  cfg.buffer = 6.0;
  LocalOrthogonalRouter r(cfg);
  r.AddBlock({1,
              {0.0, 0.0, 80.0, 80.0},
              {{1, 1.0, 0.3, kRight}, {2, 1.0, 0.7, kRight}}});
  r.AddBlock({2,
              {280.0, 40.0, 360.0, 120.0},
              {{10, 0.0, 0.3, kLeft}, {11, 0.0, 0.7, kLeft}}});
  r.AddConnector({1, 1, 1, 2, 10});
  r.AddConnector({2, 1, 2, 2, 11});
  r.RouteAllParallel(1);

  CHECK(r.MoveRouteSegment(1, 1, 220.0));
  CHECK(r.route(1).user_modified);

  const std::vector<Id> surviving =
      r.UpdateBlockPins(1, {0.0, 0.0, 80.0, 80.0}, {{1, 1.0, 0.4, kRight}});
  CHECK(surviving == std::vector<Id>{1});
  CHECK(!r.all_conns().count(2));
  CHECK(r.all_conns().count(1));
  CHECK(r.route(1).user_modified);

  (void)r.RerouteAffected({1});
  CHECK(r.route(1).status == Status::kValid);
  CHECK(r.route(1).user_modified);
  CHECK(r.route(1).points.size() >= 4);
  CHECK(std::abs(r.route(1).points[1].x - 220.0) <= 1e-6);
}

void TestAsyncIncrementalTopoFifoOrder() {
  Config cfg;
  cfg.buffer = 6.0;
  LocalOrthogonalRouter initial(cfg);
  initial.AddBlock({1, {0.0, 0.0, 80.0, 60.0}, {{1, 1.0, 0.5, kRight}}});
  initial.AddBlock({2, {240.0, 20.0, 320.0, 80.0}, {{2, 0.0, 0.5, kLeft}}});
  initial.AddConnector({1, 1, 1, 2, 2});

  AsyncLocalRouter async_router(std::move(initial));
  async_router.StartAll();
  // Interleave Move -> AddBlock -> AddConnector -> Move -> UpdateBlockPins ->
  // RemoveBlock strictly in order while the worker thread runs.
  async_router.Move({{1, {0.0, 40.0, 80.0, 100.0}}});
  async_router.AddBlock(
      {3, {0.0, 160.0, 80.0, 220.0}, {{1, 1.0, 0.5, kRight}}});
  async_router.AddBlock(
      {4, {240.0, 200.0, 320.0, 260.0}, {{2, 0.0, 0.5, kLeft}}});
  async_router.AddConnector({2, 3, 1, 4, 2});
  async_router.Move({{3, {20.0, 180.0, 100.0, 240.0}}});
  (void)async_router.UpdateBlockPins(
      1, {0.0, 50.0, 80.0, 110.0},
      {{1, 1.0, 0.5, kRight}, {9, 0.0, 0.5, kLeft}});
  async_router.RemoveBlock(2);

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (async_router.PendingCount() > 0) {
    CHECK(std::chrono::steady_clock::now() < deadline);
    if (!async_router.Poll(32)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  async_router.Poll(32);

  const auto& v = async_router.view();
  CHECK(!v.all_blocks().count(2));
  CHECK(!v.all_conns().count(1));
  CHECK(v.all_blocks().count(1) && std::abs(v.block(1).rect.t - 50.0) <= 1e-6);
  CHECK(v.all_blocks().count(3) && std::abs(v.block(3).rect.l - 20.0) <= 1e-6);
  CHECK(v.all_conns().count(2));
  CHECK(v.route(2).status == Status::kValid);
  CHECK(std::abs(v.route(2).points.front().x - 100.0) <= 1e-6);
}

void TestAsyncViewportPriorityNudge() {
  Config cfg;
  cfg.buffer = 6.0;
  cfg.nudge_distance = 8.0;
  cfg.enable_channel_nudging = true;
  LocalOrthogonalRouter initial(cfg);

  // Two visible parallel stacked routes inside viewport [0, 500]^2 with
  // overlapping vertical spans [45, 145] and [85, 185].
  initial.AddBlock({1, {0.0, 20.0, 80.0, 70.0}, {{1, 1.0, 0.5, kRight}}});
  initial.AddBlock({2, {320.0, 120.0, 400.0, 170.0}, {{2, 0.0, 0.5, kLeft}}});
  initial.AddBlock({3, {0.0, 60.0, 80.0, 110.0}, {{1, 1.0, 0.5, kRight}}});
  initial.AddBlock({4, {320.0, 160.0, 400.0, 210.0}, {{2, 0.0, 0.5, kLeft}}});
  initial.AddConnector({1, 1, 1, 2, 2});
  initial.AddConnector({2, 3, 1, 4, 2});

  // 250 offscreen blocks/wires far away so StartAll() spans multiple batches.
  Id next_bid = 100;
  Id next_cid = 100;
  for (int i = 0; i < 250; ++i) {
    const double y = static_cast<double>(i) * 120.0;
    const Id s = next_bid++;
    const Id d = next_bid++;
    initial.AddBlock(
        {s, {4000.0, y, 4080.0, y + 50.0}, {{1, 1.0, 0.5, kRight}}});
    initial.AddBlock(
        {d, {4300.0, y + 40.0, 4380.0, y + 90.0}, {{2, 0.0, 0.5, kLeft}}});
    initial.AddConnector({next_cid++, s, 1, d, 2});
  }

  AsyncLocalRouter async_router(std::move(initial));
  async_router.Prioritize({-50.0, -50.0, 500.0, 400.0});
  async_router.StartAll();

  bool verified_early_nudge = false;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (async_router.PendingCount() > 0) {
    CHECK(std::chrono::steady_clock::now() < deadline);
    if (!async_router.Poll(8)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    const auto& v = async_router.view();
    if (v.route(1).status == Status::kValid &&
        v.route(2).status == Status::kValid &&
        async_router.PendingCount() > 0) {
      CHECK(v.route(1).points.size() >= 4);
      CHECK(v.route(2).points.size() >= 4);
      const double gap =
          std::abs(v.route(1).points[1].x - v.route(2).points[1].x);
      CHECK(gap >= 4.0);
      verified_early_nudge = true;
      break;
    }
  }
  CHECK(verified_early_nudge);
}

void TestSharedTrunkSegmentDragNoNeighborPush() {
  Config cfg;
  cfg.buffer = 6.0;
  cfg.nudge_distance = 8.0;
  LocalOrthogonalRouter r(cfg);
  // Same-source fan-out from block 8 pin 200 to blocks 11 and 12 (conns 3 & 4).
  r.AddBlock({8, {0.0, 80.0, 90.0, 130.0}, {{200, 1.0, 0.5, kRight}}});
  r.AddBlock({11, {270.0, 160.0, 360.0, 210.0}, {{100, 0.0, 0.5, kLeft}}});
  r.AddBlock({12, {270.0, 240.0, 360.0, 290.0}, {{100, 0.0, 0.5, kLeft}}});
  r.AddConnector({3, 8, 200, 11, 100});
  r.AddConnector({4, 8, 200, 12, 100});

  // Unrelated nearby wire within nudge_distance of target x=210.
  r.AddBlock({20, {0.0, 0.0, 90.0, 50.0}, {{200, 1.0, 0.5, kRight}}});
  r.AddBlock({21, {334.0, 120.0, 424.0, 170.0}, {{100, 0.0, 0.5, kLeft}}});
  r.AddConnector({99, 20, 200, 21, 100});

  r.RouteAllParallel(1);
  CHECK(r.route(3).status == Status::kValid && r.route(3).points.size() == 4);
  CHECK(r.route(4).status == Status::kValid && r.route(4).points.size() == 4);
  CHECK(std::abs(r.route(3).points[1].x - r.route(4).points[1].x) <= 1e-6);
  CHECK(r.route(99).status == Status::kValid);
  const std::vector<Point> neighbor_before = r.route(99).points;

  // Move conn 3's trunk segment to x = 210 (right next to neighbor conn 99 at
  // x = 212): BOTH conn 3 and conn 4 follow together, solder joint remains,
  // and neighbor conn 99 MUST NOT move!
  std::vector<Id> changed;
  CHECK(r.MoveRouteSegment(3, 1, 210.0, &changed));
  CHECK(changed.size() == 2);
  CHECK(std::abs(r.route(3).points[1].x - 210.0) <= 1e-6);
  CHECK(std::abs(r.route(4).points[1].x - 210.0) <= 1e-6);
  CHECK(r.route(3).user_modified && r.route(4).user_modified);
  CHECK(!r.ComputeSolderJoints({3, 4}).empty());
  CHECK(r.route(99).points == neighbor_before);

  // Verify parity through AsyncLocalRouter::MoveRouteSegment.
  AsyncLocalRouter async_router(std::move(r));
  std::vector<Id> async_changed;
  CHECK(async_router.MoveRouteSegment(3, 1, 215.0, &async_changed));
  CHECK(async_changed.size() == 2);
  CHECK(std::abs(async_router.view().route(3).points[1].x - 215.0) <= 1e-6);
  CHECK(std::abs(async_router.view().route(4).points[1].x - 215.0) <= 1e-6);
  CHECK(async_router.view().route(99).points == neighbor_before);
}

void TestChannelNudgingElasticSpacing() {
  Config bad_cfg;
  bad_cfg.nudge_distance = 12.0;
  bad_cfg.max_nudge_distance = 8.0;
  bool threw = false;
  try {
    LocalOrthogonalRouter bad_router(bad_cfg);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);

  const Config cfg;
  CHECK(cfg.nudge_distance == 12.0);
  CHECK(cfg.max_nudge_distance == 36.0);

  auto run_bundle = [&](double b2_left) {
    LocalOrthogonalRouter r(cfg);
    std::vector<Pin> b1_pins;
    std::vector<Pin> b2_pins;
    for (Id i = 0; i < 10; ++i) {
      const double y = static_cast<double>(i + 1) / 11.0;
      b1_pins.push_back({100 + i, 1.0, y, kRight});
      b2_pins.push_back({200 + i, 0.0, y, kLeft});
    }
    r.AddBlock({1, {0.0, 0.0, 80.0, 260.0}, b1_pins});
    r.AddBlock({2, {b2_left, 60.0, b2_left + 80.0, 320.0}, b2_pins});
    for (Id i = 0; i < 10; ++i) {
      r.AddConnector({100 + i, 1, 100 + i, 2, 200 + i});
    }
    r.RouteAllParallel(1);

    std::vector<double> mid_xs;
    mid_xs.reserve(10);
    for (Id i = 0; i < 10; ++i) {
      const Route& rt = r.route(100 + i);
      CHECK(rt.status == Status::kValid);
      CHECK(rt.points.size() == 4);
      mid_xs.push_back(rt.points[1].x);
    }
    for (Id i = 0; i < 10; ++i) {
      const auto& pa = r.route(100 + i).points;
      for (Id j = i + 1; j < 10; ++j) {
        const auto& pb = r.route(100 + j).points;
        for (std::size_t sa = 1; sa < pa.size(); ++sa) {
          for (std::size_t sb = 1; sb < pb.size(); ++sb) {
            CHECK(
                !SegmentsProperlyCross(pa[sa - 1], pa[sa], pb[sb - 1], pb[sb]));
          }
        }
      }
    }
    std::sort(mid_xs.begin(), mid_xs.end());
    std::vector<double> gaps;
    for (std::size_t i = 1; i < mid_xs.size(); ++i) {
      gaps.push_back(mid_xs[i] - mid_xs[i - 1]);
    }
    return gaps;
  };

  // Narrow corridor (80 px gap -> avail ~63.9 < 9*10): compressed cleanly.
  const std::vector<double> narrow_gaps = run_bundle(160.0);
  const double narrow_min =
      *std::min_element(narrow_gaps.begin(), narrow_gaps.end());
  const double narrow_max =
      *std::max_element(narrow_gaps.begin(), narrow_gaps.end());
  CHECK(narrow_min > 4.0);
  CHECK(narrow_max < cfg.nudge_distance);

  // Mid-wide corridor (600 px gap -> avail ~583.9, step ~29.2): elastically
  // expands > nudge_distance and < max_nudge_distance.
  const std::vector<double> mid_gaps = run_bundle(680.0);
  const double mid_min = *std::min_element(mid_gaps.begin(), mid_gaps.end());
  const double mid_max = *std::max_element(mid_gaps.begin(), mid_gaps.end());
  CHECK(mid_min > cfg.nudge_distance + 1.0);
  CHECK(mid_max < cfg.max_nudge_distance);
  CHECK(mid_min > narrow_max);

  // Extra-wide corridor (800 px gap -> avail ~783.9): elastically expands
  // strictly to max_nudge_distance (36 px) with 0 overlaps and 0 crossings.
  const std::vector<double> wide_gaps = run_bundle(880.0);
  const double wide_min = *std::min_element(wide_gaps.begin(), wide_gaps.end());
  const double wide_max = *std::max_element(wide_gaps.begin(), wide_gaps.end());
  CHECK(std::abs(wide_min - cfg.max_nudge_distance) <= 1e-3);
  CHECK(std::abs(wide_max - cfg.max_nudge_distance) <= 1e-3);
  CHECK(wide_min > mid_max);
}

class FixedMultiSegSolver final : public LocalRouteSolver {
 public:
  const char* name() const override { return "FixedMultiSegSolver"; }
  std::vector<Point> Solve(const Request& r) const override {
    if (r.conn.id == 1) {
      return {{40.0, 20.0},  {108.0, 20.0},  {108.0, 60.0},
              {120.0, 60.0}, {120.0, 120.0}, {200.0, 120.0}};
    }
    return inner_->Solve(r);
  }

 private:
  std::shared_ptr<const LocalRouteSolver> inner_ = LazyHananAStarSolver();
};

void TestSegmentIndexStateMachineAndShrink() {
  // 1. DisplayOnly + kReuse followed by AutoRouteConnectors & RemoveConnector.
  {
    Config cfg;
    cfg.buffer = 6.0;
    cfg.reroute_third_party_blocked = true;
    LocalOrthogonalRouter r(cfg);
    r.AddBlock({1, {0.0, 20.0, 60.0, 60.0}, {{1, 1.0, 0.5, kRight}}});
    r.AddBlock({2, {300.0, 20.0, 360.0, 60.0}, {{2, 0.0, 0.5, kLeft}}});
    r.AddBlock({3, {160.0, 10.0, 200.0, 70.0}, {}});
    r.AddConnector({1, 1, 1, 2, 2});
    r.RouteAllParallel(1);
    CHECK(r.route(1).status == Status::kValid);
    CHECK(r.route(1).points.size() >= 6);

    CHECK(r.MoveRouteSegment(1, 2, 40.0));
    CHECK(r.route(1).status == Status::kDisplayOnly);
    CHECK(r.route(1).method == RouteMethod::kReuse);

    const std::vector<Id> auto_changed = r.AutoRouteConnectors({1});
    CHECK(!auto_changed.empty());
    CHECK(r.route(1).status == Status::kValid);

    CHECK(r.MoveRouteSegment(1, 2, 40.0));
    CHECK(r.route(1).status == Status::kDisplayOnly);
    r.RemoveConnector(1);
    CHECK(r.all_conns().empty());
  }

  // 2. MarkConnector -> enable reroute_third_party_blocked -> Route/Reroute.
  {
    Config cfg;
    cfg.buffer = 6.0;
    LocalOrthogonalRouter r(cfg);
    r.AddBlock({1, {0.0, 0.0, 60.0, 40.0}, {{1, 1.0, 0.5, kRight}}});
    r.AddBlock({2, {200.0, 40.0, 260.0, 80.0}, {{2, 0.0, 0.5, kLeft}}});
    r.AddConnector({1, 1, 1, 2, 2});
    r.RouteAllParallel(1);
    CHECK(r.route(1).status == Status::kValid);

    r.MarkConnector(1);
    CHECK(r.route(1).status == Status::kPending);
    r.set_reroute_third_party_blocked(true);
    r.RouteConnector(1);
    CHECK(r.route(1).status == Status::kValid);

    r.MarkConnector(1);
    (void)r.RerouteAffected({});
    CHECK(r.route(1).status == Status::kValid);
  }

  // 3. Segment count shrink via ChannelNudger removes stale segment keys.
  {
    Config cfg;
    cfg.buffer = 4.0;
    cfg.nudge_distance = 12.0;
    cfg.fast_paths = false;
    cfg.reroute_third_party_blocked = true;
    cfg.enable_channel_nudging = false;
    LocalOrthogonalRouter r(cfg, std::make_shared<FixedMultiSegSolver>());
    r.AddBlock({1, {0.0, 0.0, 40.0, 40.0}, {{1, 1.0, 0.5, kRight}}});
    r.AddBlock({2, {200.0, 100.0, 240.0, 140.0}, {{2, 0.0, 0.5, kLeft}}});
    r.AddBlock({3, {0.0, -60.0, 40.0, -20.0}, {{1, 1.0, 0.5, kRight}}});
    r.AddBlock({4, {200.0, 160.0, 240.0, 200.0}, {{2, 0.0, 0.5, kLeft}}});
    r.AddConnector({1, 1, 1, 2, 2});
    r.AddConnector({2, 3, 1, 4, 2});

    r.RouteConnector(1);
    CHECK(r.route(1).status == Status::kValid);
    CHECK(r.route(1).points.size() == 6);

    Route anchor;
    anchor.status = Status::kValid;
    anchor.method = RouteMethod::kReuse;
    anchor.user_modified = true;
    anchor.points = {
        {40.0, -40.0}, {120.0, -40.0}, {120.0, 180.0}, {200.0, 180.0}};
    r.RestoreRoute(2, std::move(anchor));

    r.set_enable_channel_nudging(true);
    r.RouteConnector(1);
    CHECK(r.route(1).status == Status::kValid);
    CHECK(r.route(1).points.size() == 4);
    CHECK(std::abs(r.route(1).points[1].x - 109.0) <= 1e-6);

    // Adding an obstacle overlapping only the old (x=120, y:[60,120]) segment
    // must not invalidate conn 1 whose simplified segment moved to x=109.
    r.AddBlock({99, {122.0, 80.0, 140.0, 100.0}, {}});
    CHECK(r.route(1).status == Status::kValid);

    r.RemoveConnector(1);
    CHECK(!r.all_conns().count(1));
  }
}

void TestAsyncThirdPartySegmentsSync() {
  auto drain = [](AsyncLocalRouter* ar) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (ar->PendingCount() > 0) {
      CHECK(std::chrono::steady_clock::now() < deadline);
      if (!ar->Poll(64)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    ar->Poll(64);
  };

  // 1. Construct AsyncLocalRouter from pre-routed model with
  //    reroute_third_party_blocked = true.
  {
    Config cfg;
    cfg.buffer = 6.0;
    cfg.reroute_third_party_blocked = true;
    LocalOrthogonalRouter initial(cfg);
    initial.AddBlock({1, {0.0, 0.0, 60.0, 40.0}, {{1, 1.0, 0.5, kRight}}});
    initial.AddBlock({2, {240.0, 0.0, 300.0, 40.0}, {{2, 0.0, 0.5, kLeft}}});
    initial.AddBlock({3, {120.0, 120.0, 180.0, 160.0}, {}});
    initial.AddConnector({1, 1, 1, 2, 2});
    initial.RouteAllParallel(1);
    CHECK(initial.route(1).status == Status::kValid);

    AsyncLocalRouter ar(std::move(initial));
    ar.Move({{3, {120.0, 0.0, 180.0, 40.0}}});
    drain(&ar);
    CHECK(ar.view().route(1).status == Status::kValid);
    CHECK(ar.view().route(1).points.size() >= 4);
  }

  // 2. Enable reroute_third_party_blocked at runtime after StartAll().
  {
    Config cfg;
    cfg.buffer = 6.0;
    cfg.reroute_third_party_blocked = false;
    LocalOrthogonalRouter initial(cfg);
    initial.AddBlock({1, {0.0, 0.0, 60.0, 40.0}, {{1, 1.0, 0.5, kRight}}});
    initial.AddBlock({2, {240.0, 0.0, 300.0, 40.0}, {{2, 0.0, 0.5, kLeft}}});
    initial.AddBlock({3, {120.0, 120.0, 180.0, 160.0}, {}});
    initial.AddConnector({1, 1, 1, 2, 2});

    AsyncLocalRouter ar(std::move(initial));
    ar.StartAll();
    drain(&ar);
    CHECK(ar.view().route(1).status == Status::kValid);

    ar.set_reroute_third_party_blocked(true);
    ar.Move({{3, {120.0, 0.0, 180.0, 40.0}}});
    drain(&ar);
    CHECK(ar.view().route(1).status == Status::kValid);
    CHECK(ar.view().route(1).points.size() >= 4);
  }
}

void TestAsyncRestorePendingRouteConverges() {
  Config cfg;
  cfg.buffer = 6.0;
  LocalOrthogonalRouter initial(cfg);
  initial.AddBlock({1, {0.0, 0.0, 80.0, 60.0}, {{1, 1.0, 0.5, kRight}}});
  initial.AddBlock({2, {240.0, 40.0, 320.0, 100.0}, {{2, 0.0, 0.5, kLeft}}});

  AsyncLocalRouter ar(std::move(initial));
  ar.AddConnector({1, 1, 1, 2, 2});
  CHECK(ar.PendingCount() == 1);
  const Route pending_snap = ar.view().route(1);
  CHECK(pending_snap.status == Status::kPending);

  const std::vector<Id> changed = ar.AutoRouteConnectors({1});
  CHECK(changed.size() == 1);
  CHECK(ar.view().route(1).status == Status::kValid);
  CHECK(ar.PendingCount() == 0);

  ar.RestoreRoute(1, pending_snap);
  CHECK(ar.view().route(1).status == Status::kPending);
  CHECK(ar.PendingCount() > 0);

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (ar.PendingCount() > 0) {
    CHECK(std::chrono::steady_clock::now() < deadline);
    if (!ar.Poll(64)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  ar.Poll(64);
  CHECK(ar.view().route(1).status == Status::kValid);
  CHECK(ar.view().route(1).points.size() >= 4);
}

void TestAsyncLargeBatchZeroDisturbance() {
  const Topology top = GenerateTopology(1000);
  Config cfg;
  cfg.enable_channel_nudging = true;
  LocalOrthogonalRouter initial(cfg);
  for (const auto& b : top.blocks) {
    initial.AddBlock(b);
  }
  for (const auto& c : top.conns) {
    initial.AddConnector(c);
  }

  // Remote hub block with 257 connectors (> 256) placed far away from the
  // 1000-block topology.
  const Id hub_id = 20000;
  std::vector<Pin> hub_pins;
  hub_pins.reserve(257);
  for (Id p = 1; p <= 257; ++p) {
    hub_pins.push_back({p, 1.0, static_cast<double>(p) / 258.0, kRight});
  }
  initial.AddBlock({hub_id, {30000.0, 0.0, 30100.0, 2600.0}, hub_pins});
  for (Id i = 0; i < 257; ++i) {
    const Id leaf_id = 20001 + i;
    const double y = static_cast<double>(i) * 20.0;
    initial.AddBlock(
        {leaf_id, {30300.0, y, 30380.0, y + 12.0}, {{1, 0.0, 0.5, kLeft}}});
    initial.AddConnector({50000 + i, hub_id, i + 1, leaf_id, 1});
  }

  AsyncLocalRouter ar(std::move(initial));
  ar.StartAll();
  auto drain = [&]() {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (ar.PendingCount() > 0) {
      CHECK(std::chrono::steady_clock::now() < deadline);
      if (!ar.Poll(256)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    ar.Poll(256);
  };
  drain();

  std::vector<std::vector<Point>> saved_points;
  saved_points.reserve(top.conns.size());
  for (const auto& c : top.conns) {
    saved_points.push_back(ar.view().route(c.id).points);
  }

  // Move the 257-connector hub; must stay in incremental full_graph = false
  // mode and leave all 2,000 original topology routes completely untouched.
  ar.Move({{hub_id, {30000.0, 20.0, 30100.0, 2620.0}}});
  drain();

  for (std::size_t i = 0; i < top.conns.size(); ++i) {
    CHECK(ar.view().route(top.conns[i].id).points == saved_points[i]);
  }
}

void TestAsyncNudgeSyncBackAndPriorityRace() {
  auto drain = [](AsyncLocalRouter* ar) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (ar->PendingCount() > 0) {
      CHECK(std::chrono::steady_clock::now() < deadline);
      if (!ar->Poll(64)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    ar->Poll(64);
  };

  // Part A: after async nudging & deleting wire 2, translating wire 1 keeps
  // its nudged track instead of snapping back to the raw midline (x = 200).
  {
    Config cfg;
    cfg.buffer = 6.0;
    cfg.nudge_distance = 12.0;
    cfg.enable_channel_nudging = true;
    LocalOrthogonalRouter initial(cfg);
    initial.AddBlock({1, {0.0, 0.0, 80.0, 50.0}, {{1, 1.0, 0.5, kRight}}});
    initial.AddBlock({2, {320.0, 100.0, 400.0, 150.0}, {{2, 0.0, 0.5, kLeft}}});
    initial.AddBlock({3, {0.0, 70.0, 80.0, 120.0}, {{1, 1.0, 0.5, kRight}}});
    initial.AddBlock({4, {320.0, 170.0, 400.0, 220.0}, {{2, 0.0, 0.5, kLeft}}});
    initial.AddConnector({1, 1, 1, 2, 2});
    initial.AddConnector({2, 3, 1, 4, 2});

    AsyncLocalRouter ar(std::move(initial));
    ar.StartAll();
    drain(&ar);

    CHECK(ar.view().route(1).status == Status::kValid);
    CHECK(ar.view().route(2).status == Status::kValid);
    CHECK(ar.view().route(1).points.size() == 4);
    const double nudged_x1 = ar.view().route(1).points[1].x;
    CHECK(std::abs(nudged_x1 - 200.0) > 1.0);

    ar.RemoveConnector(2);
    ar.Move({{1, {0.0, 10.0, 80.0, 60.0}}, {2, {320.0, 110.0, 400.0, 160.0}}});
    drain(&ar);

    CHECK(ar.view().route(1).status == Status::kValid);
    CHECK(ar.view().route(1).points.size() == 4);
    CHECK(std::abs(ar.view().route(1).points[1].x - nudged_x1) <= 1e-6);
  }

  // Part B: priority_wave_finished sync-back followed immediately by Move,
  // RemoveConnector, AddConnector (same ID & new ID) must never clobber new
  // edits with stale nudged routes.
  {
    Config cfg;
    cfg.buffer = 6.0;
    cfg.nudge_distance = 12.0;
    cfg.enable_channel_nudging = true;
    LocalOrthogonalRouter initial(cfg);
    initial.AddBlock({1, {0.0, 0.0, 80.0, 50.0}, {{1, 1.0, 0.5, kRight}}});
    initial.AddBlock({2, {320.0, 100.0, 400.0, 150.0}, {{2, 0.0, 0.5, kLeft}}});
    initial.AddBlock({3, {0.0, 70.0, 80.0, 120.0}, {{1, 1.0, 0.5, kRight}}});
    initial.AddBlock({4, {320.0, 170.0, 400.0, 220.0}, {{2, 0.0, 0.5, kLeft}}});
    initial.AddConnector({1, 1, 1, 2, 2});
    initial.AddConnector({2, 3, 1, 4, 2});

    Id next_bid = 100;
    Id next_cid = 100;
    for (int i = 0; i < 300; ++i) {
      const double y = static_cast<double>(i) * 120.0;
      const Id s = next_bid++;
      const Id d = next_bid++;
      initial.AddBlock(
          {s, {5000.0, y, 5080.0, y + 50.0}, {{1, 1.0, 0.5, kRight}}});
      initial.AddBlock(
          {d, {5300.0, y + 40.0, 5380.0, y + 90.0}, {{2, 0.0, 0.5, kLeft}}});
      initial.AddConnector({next_cid++, s, 1, d, 2});
    }

    AsyncLocalRouter ar(std::move(initial));
    ar.Prioritize({-50.0, -50.0, 450.0, 350.0});
    ar.StartAll();

    bool triggered_pri_nudge = false;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (ar.PendingCount() > 0) {
      CHECK(std::chrono::steady_clock::now() < deadline);
      if (!ar.Poll(2)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      const auto& v = ar.view();
      if (v.route(1).status == Status::kValid &&
          v.route(2).status == Status::kValid && ar.PendingCount() > 0) {
        const double gap =
            std::abs(v.route(1).points[1].x - v.route(2).points[1].x);
        CHECK(gap >= 4.0);
        triggered_pri_nudge = true;
        break;
      }
    }
    CHECK(triggered_pri_nudge);

    ar.Move({{1, {0.0, 15.0, 80.0, 65.0}}});
    ar.RemoveConnector(2);
    ar.AddBlock({5, {320.0, 260.0, 400.0, 310.0}, {{2, 0.0, 0.5, kLeft}}});
    ar.AddConnector({2, 3, 1, 5, 2});
    ar.AddConnector({9999, 1, 1, 5, 2});
    drain(&ar);

    const auto& v = ar.view();
    CHECK(v.route(1).status == Status::kValid);
    CHECK(std::abs(v.route(1).points.front().y - 40.0) <= 1e-6);
    CHECK(v.route(2).status == Status::kValid);
    CHECK(std::abs(v.route(2).points.back().y - 285.0) <= 1e-6);
    CHECK(v.route(9999).status == Status::kValid);
    CHECK(std::abs(v.route(9999).points.front().y - 40.0) <= 1e-6);
    CHECK(std::abs(v.route(9999).points.back().y - 285.0) <= 1e-6);
  }
}

void TestSameBlockSelfLoopWithBuffer() {
  Config cfg;
  cfg.buffer = 8.0;
  LocalOrthogonalRouter r(cfg);
  const Rect blk{0.0, 0.0, 100.0, 60.0};
  r.AddBlock({1,
              blk,
              {{1, 1.0, 0.5, kRight},
               {2, 0.0, 0.5, kLeft},
               {3, 1.0, 0.25, kRight},
               {4, 1.0, 0.75, kRight}}});
  r.AddConnector({1, 1, 1, 1, 2});
  r.AddConnector({2, 1, 3, 1, 4});
  r.RouteAllParallel(1);

  for (Id cid : {Id{1}, Id{2}}) {
    const Route& rt = r.route(cid);
    CHECK(rt.status == Status::kValid);
    CHECK(rt.points.size() >= 4);
    for (std::size_t i = 1; i < rt.points.size(); ++i) {
      const Point& u = rt.points[i - 1];
      const Point& v = rt.points[i];
      CHECK(u.x == v.x || u.y == v.y);
      CHECK(!(u == v));
      const Rect s = Box(u, v);
      CHECK(!(s.r > blk.l && s.l < blk.r && s.b > blk.t && s.t < blk.b));
    }
  }
  CHECK(r.route(1).points[1].x > blk.r);
  CHECK(r.route(1).points[r.route(1).points.size() - 2].x < blk.l);
}

void TestExportRegionToSvgUtf8Path() {
  LocalOrthogonalRouter r;
  r.AddBlock({1, {0.0, 0.0, 80.0, 60.0}, {{1, 1.0, 0.5, kRight}}});
  r.AddBlock({2, {200.0, 0.0, 280.0, 60.0}, {{2, 0.0, 0.5, kLeft}}});
  r.AddConnector({1, 1, 1, 2, 2});
  r.RouteAllParallel(1);

  const std::filesystem::path target =
      std::filesystem::temp_directory_path() /
      std::filesystem::u8path(
          "localroute_\xE6\xB5\x8B\xE8\xAF\x95_\xF0\x9F\x94\x8C.svg");
  std::filesystem::remove(target);
  r.ExportRegionToSvg(target.u8string(), {-20.0, -20.0, 300.0, 100.0});
  CHECK(std::filesystem::exists(target));
  {
    std::ifstream ifs(target);
    CHECK(ifs.is_open());
    const std::string content((std::istreambuf_iterator<char>(ifs)),
                              std::istreambuf_iterator<char>());
    CHECK(content.find("<svg") != std::string::npos);
    CHECK(content.find("</svg>") != std::string::npos);
  }
  std::filesystem::remove(target);
}

void TestLocalNudgeThirdPartySegmentsLookup() {
  Config cfg;
  cfg.buffer = 6.0;
  cfg.nudge_distance = 12.0;
  cfg.reroute_third_party_blocked = true;
  cfg.enable_channel_nudging = true;
  LocalOrthogonalRouter r(cfg);

  // Passing long-span wire whose endpoints sit at x = -2000 and x = 2000, so
  // its vertical middle segment at x = 30 passes through a local corridor far
  // from both endpoint blocks.
  r.AddBlock({1, {-2060.0, 0.0, -1940.0, 40.0}, {{1, 1.0, 0.5, kRight}}});
  r.AddBlock({2, {2000.0, 200.0, 2060.0, 240.0}, {{2, 0.0, 0.5, kLeft}}});
  r.AddConnector({1, 1, 1, 2, 2});
  r.RouteAllParallel(1);
  CHECK(r.route(1).status == Status::kValid);
  CHECK(r.route(1).points.size() == 4);
  const std::vector<Point> long_before = r.route(1).points;
  const double long_mid_x = long_before[1].x;
  CHECK(std::abs(long_mid_x - 30.0) <= 1e-6);

  // Local wire whose default midline also lands on x = 30.0.
  r.AddBlock({3, {-60.0, 60.0, 0.0, 100.0}, {{1, 1.0, 0.5, kRight}}});
  r.AddBlock({4, {60.0, 120.0, 120.0, 160.0}, {{2, 0.0, 0.5, kLeft}}});
  r.AddConnector({2, 3, 1, 4, 2});
  r.RouteConnector(2);

  CHECK(r.route(1).points == long_before);
  CHECK(r.route(2).status == Status::kValid);
  CHECK(r.route(2).points.size() == 4);
  CHECK(std::abs(r.route(2).points[1].x - long_mid_x) >=
        cfg.nudge_distance - 1e-3);
}

void TestRestoreRouteEndpointValidation() {
  // 1. Sync router downgrades misaligned or short kValid routes to kPending.
  {
    LocalOrthogonalRouter r;
    AddPair(&r);
    r.RouteAllParallel(1);
    CHECK(r.route(1).status == Status::kValid);
    const Route saved = r.route(1);

    r.MoveBlock(1, 0.0, 20.0);
    r.RestoreRoute(1, saved);
    CHECK(r.route(1).status == Status::kPending);

    const std::vector<Id> rerouted = r.RerouteAffected({});
    CHECK(rerouted == std::vector<Id>{1});
    CHECK(r.route(1).status == Status::kValid);
    CHECK(std::abs(r.route(1).points.front().y - 40.0) <= 1e-6);

    Route short_valid = saved;
    short_valid.points = {{40.0, 40.0}};
    r.RestoreRoute(1, std::move(short_valid));
    CHECK(r.route(1).status == Status::kPending);
  }

  // 2. Async router re-queues downgraded kValid snapshots and converges.
  {
    LocalOrthogonalRouter initial;
    AddPair(&initial);
    initial.RouteAllParallel(1);
    const Route saved = initial.route(1);

    AsyncLocalRouter ar(std::move(initial));
    ar.Move({{1, {0.0, 20.0, 40.0, 60.0}}});
    auto drain = [&]() {
      const auto deadline =
          std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (ar.PendingCount() > 0) {
        CHECK(std::chrono::steady_clock::now() < deadline);
        if (!ar.Poll(32)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }
      ar.Poll(32);
    };
    drain();
    CHECK(ar.view().route(1).status == Status::kValid);

    ar.RestoreRoute(1, saved);
    CHECK(ar.view().route(1).status == Status::kPending);
    CHECK(ar.PendingCount() > 0);
    drain();
    CHECK(ar.view().route(1).status == Status::kValid);
    CHECK(std::abs(ar.view().route(1).points.front().y - 40.0) <= 1e-6);
  }
}

void TestSpatialGridCrossRegionDuplicateCheck() {
  SpatialGridIndex g(100.0);
  const Rect small_rect{0.0, 0.0, 50.0, 50.0};
  const Rect large_rect{0.0, 0.0, 4000.0, 4000.0};

  auto expect_dup = [&](auto fn) {
    bool threw = false;
    try {
      fn();
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
  };

  // boxes_ -> boxes_ and boxes_ -> overflow_
  g.Insert(1, small_rect, false);
  expect_dup([&] { g.Insert(1, small_rect, false); });
  expect_dup([&] { g.Insert(1, large_rect, true); });

  // overflow_ -> overflow_ and overflow_ -> boxes_
  g.Insert(2, large_rect, true);
  expect_dup([&] { g.Insert(2, large_rect, true); });
  expect_dup([&] { g.Insert(2, small_rect, false); });

  g.Remove(1);
  g.Remove(2);
  g.Insert(1, large_rect, true);
  g.Insert(2, small_rect, false);
  CHECK(g.Query({10.0, 10.0, 20.0, 20.0}) == std::vector<Id>({1, 2}));
}

}  // namespace
}  // namespace localroute

int main(int argc, char** argv) {
  const std::pair<std::string, void (*)()> kCases[] = {
      {"SpatialGridBasic", localroute::TestGrid},
      {"DirtyOnlyNoRerouteOnOverlap", localroute::TestDirty},
      {"LocalObstacleAvoidance", localroute::TestAvoid},
      {"AdaptiveWindowExpansion", localroute::TestExpansion},
      {"OverlappingBlocksFallback", localroute::TestOverlap},
      {"ParallelColdStartDeterminism", localroute::TestParallel},
      {"FastReuseAndThirdPartyObstruction", localroute::TestFastAndObstruction},
      {"AsyncDiscardsStaleResults", localroute::TestAsyncRouting},
      {"LazyHananAStarBoundaryCases", localroute::TestBoundaryCases},
      {"ChannelNudgingParallelBundle",
       localroute::TestChannelNudgingParallelBundle},
      {"ChannelNudgingCascadeAndNoHook",
       localroute::TestChannelNudgingCascadeAndNoHook},
      {"NarrowGapObstacleAvoidance", localroute::TestNarrowGapAvoidance},
      {"SolderJointDetection", localroute::TestSolderJointDetection},
      {"ManualSegmentDragAndPreserve",
       localroute::TestManualSegmentDragAndPreserve},
      {"AutoRouteSelectedLines", localroute::TestAutoRouteSelectedLines},
      {"WireCongestionSoftPenalty", localroute::TestWireCongestionSoftPenalty},
      {"UpdateBlockPinsPreservesValidWires",
       localroute::TestUpdateBlockPinsPreservesValidWires},
      {"AsyncIncrementalTopoFifoOrder",
       localroute::TestAsyncIncrementalTopoFifoOrder},
      {"AsyncViewportPriorityNudge",
       localroute::TestAsyncViewportPriorityNudge},
      {"SharedTrunkSegmentDragNoNeighborPush",
       localroute::TestSharedTrunkSegmentDragNoNeighborPush},
      {"ChannelNudgingElasticSpacing",
       localroute::TestChannelNudgingElasticSpacing},
      {"SegmentIndexStateMachineAndShrink",
       localroute::TestSegmentIndexStateMachineAndShrink},
      {"AsyncThirdPartySegmentsSync",
       localroute::TestAsyncThirdPartySegmentsSync},
      {"AsyncRestorePendingRouteConverges",
       localroute::TestAsyncRestorePendingRouteConverges},
      {"AsyncLargeBatchZeroDisturbance",
       localroute::TestAsyncLargeBatchZeroDisturbance},
      {"AsyncNudgeSyncBackAndPriorityRace",
       localroute::TestAsyncNudgeSyncBackAndPriorityRace},
      {"SameBlockSelfLoopWithBuffer",
       localroute::TestSameBlockSelfLoopWithBuffer},
      {"ExportRegionToSvgUtf8Path", localroute::TestExportRegionToSvgUtf8Path},
      {"LocalNudgeThirdPartySegmentsLookup",
       localroute::TestLocalNudgeThirdPartySegmentsLookup},
      {"RestoreRouteEndpointValidation",
       localroute::TestRestoreRouteEndpointValidation},
      {"SpatialGridCrossRegionDuplicateCheck",
       localroute::TestSpatialGridCrossRegionDuplicateCheck},
  };
  try {
    for (const auto& t : kCases) {
      if (argc == 1 || t.first == argv[1]) {
        t.second();
        std::cout << t.first << " PASS\n";
        if (argc > 1) {
          return 0;
        }
      }
    }
    return argc == 1 ? 0 : 2;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
