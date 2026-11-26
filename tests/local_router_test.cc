#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "localroute/async_local_router.h"

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
