#include "localroute/AsyncLocalRouter.h"
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <random>

using namespace localroute;
#define CHECK(x)                                                                                             \
    do {                                                                                                     \
        if (!(x))                                                                                            \
            throw std::runtime_error(#x " at line " + std::to_string(__LINE__));                             \
    } while (0)

Block block(Id id, Rect r) { return {id, r, {{1, 1, .5, Right}, {2, 0, .5, Left}}}; }

void pair(LocalOrthogonalRouter &r) {
    r.addBlock(block(1, {0, 0, 40, 40}));
    r.addBlock(block(2, {200, 0, 240, 40}));
    r.addConnector({1, 1, 1, 2, 2});
}

struct Spy : ILocalRouteSolver {
    std::shared_ptr<const ILocalRouteSolver> inner = lazyHananAStarSolver();
    mutable std::vector<Id> seen;
    mutable bool nudge = false;
    const char *name() const override { return "spy"; }
    std::vector<Point> solve(const Request &r) const override {
        seen.push_back(r.conn.id);
        nudge = r.nudge;
        return inner->solve(r);
    }
};

void grid() {
    SpatialGridIndex g(100);
    g.insert(99999, {-30, -30, -20, -20});
    g.update(99999, {-15, -15, -10, -10});
    CHECK(g.query({-30, -30, -20, -20}).empty());
    CHECK(g.query({-10, -10, -10, -10}) == std::vector<Id>{99999});
    g.remove(99999);
    std::mt19937 rng(42);
    std::vector<Rect> v;
    for (Id i = 0; i < 10000; ++i) {
        double x = int(rng() % 8000) - 4000, y = int(rng() % 8000) - 4000;
        v.push_back({x, y, x + 50, y + 60});
        g.insert(i, v.back());
    }
    for (Id i = 0; i < 10000; i += 2) {
        v[i] = v[i].pad(40);
        g.update(i, v[i]);
    }
    for (Id i = 0; i < 10000; i += 3)
        g.remove(i);
    for (int j = 0; j < 100; ++j) {
        double x = int(rng() % 8000) - 4000, y = int(rng() % 8000) - 4000;
        Rect q{x, y, x + 350, y + 400};
        std::vector<Id> want;
        for (Id i = 0; i < 10000; ++i)
            if (i % 3 && v[i].intersects(q))
                want.push_back(i);
        CHECK(g.query(q) == want);
    }
}

void dirty() {
    auto s = std::make_shared<Spy>();
    Config cfg;
    CHECK(!cfg.rerouteThirdPartyBlocked);
    cfg.settleNudge = true;
    cfg.fastPaths = false;
    LocalOrthogonalRouter r(cfg, s);
    pair(r); // route 1: (40,20) -> (200,20)
    r.addBlock(block(3, {100, 80, 140, 120}));
    r.addBlock(block(4, {300, 80, 340, 120}));
    r.addConnector({2, 3, 1, 4, 2});
    r.routeAllParallel(1);
    auto old1 = r.route(1).points;
    CHECK(r.route(1).status == Status::Valid && old1.size() >= 2);
    s->seen.clear();
    // Move block 3 directly over the middle of connector 1 (A-B).
    // With rerouteThirdPartyBlocked == false, connector 1 must NOT become Pending or be rerouted.
    r.moveBlocksBatch({{3, {100, 5, 140, 45}}});
    CHECK(r.route(1).status == Status::Valid);
    CHECK(r.route(2).status == Status::Pending);
    CHECK(r.rerouteAffected({3, 3}) == std::vector<Id>{2});
    CHECK(s->seen == std::vector<Id>{2});
    CHECK(r.route(1).points == old1);
    s->seen.clear();
    r.rerouteAffected({3}, true);
    CHECK(s->nudge && s->seen == std::vector<Id>{2});
    r.removeBlock(3);
    r.moveBlock(4, 0, 5);
    CHECK(r.rerouteAffected({4}).empty());
}

void avoid() {
    Config cfg;
    cfg.fastPaths = false;
    LocalOrthogonalRouter r(cfg);
    pair(r);
    r.addBlock(block(3, {100, -10, 140, 50}));
    r.addBlock(block(4, {220, 120, 260, 160}));
    r.addConnector({2, 1, 1, 4, 2});
    r.routeAllParallel(1);
    for (Id id : {1u, 2u}) {
        auto a = r.route(id);
        CHECK(a.status == Status::Valid);
        CHECK(a.points.size() >= 4);
        CHECK(a.points[1].x > a.points[0].x);
        CHECK(a.points[a.points.size() - 2].x < a.points.back().x);
        for (size_t j = 1; j < a.points.size(); ++j) {
            auto s = box(a.points[j - 1], a.points[j]);
            CHECK(s.l == s.r || s.t == s.b);
            CHECK(!(s.l < 148 && s.r > 92 && s.t < 58 && s.b > -18));
        }
    }
}

struct Wall : ILocalRouteSolver {
    const char *name() const override { return "wall-test"; }
    std::vector<Point> solve(const Request &q) const override {
        for (auto o : q.obstacles)
            if (o->id == 9)
                return {{40, 20}, {45, 20}, {45, -80}, {195, -80}, {195, 20}, {200, 20}};
        return {{40, 20}, {200, 20}};
    }
};

void expansion() {
    Config cfg;
    cfg.buffer = 4;
    cfg.padding = 10;
    cfg.fastPaths = false;
    cfg.budget = 3;
    auto s = std::make_shared<Wall>();
    LocalOrthogonalRouter r(cfg, s);
    pair(r);
    r.addBlock(block(8, {75, 26, 85, 34}));
    r.addBlock(block(9, {110, -65, 125, 35}));
    r.routeAllParallel(1);
    auto a = r.route(1);
    CHECK(a.status == Status::Valid);
    CHECK(a.retries == 1 && a.calls == 2 && a.window == 3);
    cfg.budget = 2;
    LocalOrthogonalRouter limited(cfg, s);
    pair(limited);
    limited.addBlock(block(9, {110, -65, 125, 35}));
    limited.routeAllParallel(1);
    CHECK(limited.route(1).status == Status::DisplayOnly);
}

void overlap() {
    LocalOrthogonalRouter r;
    pair(r);
    r.moveBlocksBatch({{2, {0, 0, 40, 40}}});
    r.routeAllParallel(1);
    CHECK(r.route(1).status != Status::Valid);
    r.moveBlocksBatch({{2, {40, 0, 80, 40}}});
    r.routeAllParallel(1);
    CHECK(r.route(1).status == Status::Failed);
    r.moveBlocksBatch({{2, {200, 0, 240, 40}}});
    r.addBlock(block(3, {30, 10, 65, 30}));
    r.routeAllParallel(1);
    CHECK(r.route(1).status == Status::DisplayOnly && r.route(1).calls == 0);
}

void parallel() {
    LocalOrthogonalRouter r;
    for (Id i = 0; i < 500; ++i) {
        double x = i % 25 * 160., y = i / 25 * 90.;
        r.addBlock(block(i, {x, y, x + 70, y + 40}));
        if (i % 25)
            r.addConnector({i, i - 1, 1, i, 2});
    }
    r.routeAllParallel(1);
    std::vector<Route> ref(500);
    for (Id i = 1; i < 500; ++i)
        if (i % 25) {
            ref[i] = r.route(i);
            CHECK(ref[i].status == Status::Valid);
        }
    for (int rep = 0; rep < 4; ++rep) {
        r.routeAllParallel(4);
        for (Id i = 1; i < 500; ++i)
            if (i % 25) {
                auto &q = r.route(i);
                CHECK(q.status == ref[i].status && q.points == ref[i].points);
            }
    }
}

void fastAndObstruction() {
    auto s = std::make_shared<Spy>();
    Config cfg;
    cfg.rerouteThirdPartyBlocked = true;
    LocalOrthogonalRouter r(cfg, s);
    pair(r);
    r.addBlock(block(3, {100, 80, 140, 120}));
    r.routeAllParallel(1);
    CHECK(r.route(1).status == Status::Valid && r.route(1).method == RouteMethod::Simple);
    r.moveBlocksBatch({{1, {0, 5, 40, 45}}, {2, {200, 5, 240, 45}}});
    CHECK(r.route(1).status == Status::Pending);
    CHECK(r.rerouteAffected({1, 2}) == std::vector<Id>{1});
    CHECK(r.route(1).method == RouteMethod::Reuse && s->seen.empty());
    r.moveBlock(3, 0, -80);
    CHECK(r.route(1).status == Status::Pending);
    CHECK(r.rerouteAffected({3}) == std::vector<Id>{1});
    CHECK(r.route(1).status == Status::Valid && r.route(1).method == RouteMethod::Native);
    CHECK(s->seen == std::vector<Id>{1});
    r.moveBlock(1, 0, 2);
    CHECK(r.rerouteAffected({1}) == std::vector<Id>{1});
    CHECK(r.route(1).status == Status::Valid && r.route(1).method == RouteMethod::Reuse);
    CHECK(s->seen == std::vector<Id>{1});
    r.removeConnector(1);
    CHECK(r.rerouteAffected({3}).empty());
}

struct PausedSolver : ILocalRouteSolver {
    std::shared_ptr<const ILocalRouteSolver> inner = lazyHananAStarSolver();
    mutable std::promise<void> entered;
    std::shared_future<void> release;
    mutable unsigned calls = 0;
    explicit PausedSolver(std::shared_future<void> release) : release(std::move(release)) {}
    const char *name() const override { return "paused"; }
    std::vector<Point> solve(const Request &q) const override {
        if (++calls == 1) {
            entered.set_value();
            release.wait();
        }
        return inner->solve(q);
    }
};

void asyncRouting() {
    using namespace std::chrono_literals;
    std::promise<void> release;
    auto s = std::make_shared<PausedSolver>(release.get_future().share());
    auto entered = s->entered.get_future();
    Config cfg;
    cfg.fastPaths = false;
    LocalOrthogonalRouter r(cfg, s);
    pair(r);
    AsyncLocalRouter router(std::move(r));
    router.startAll();
    auto first = entered.wait_for(5s);
    auto pre = router.view().route(1).status;
    router.move({{1, {0, 4, 40, 44}}});
    router.move({{1, {0, 16, 40, 56}}});
    auto before = router.poll();
    auto ghost = router.view().preview(1).front();
    router.settle();
    release.set_value();
    CHECK(first == std::future_status::ready && pre == Status::Pending && before == 0 && (ghost == Point{40, 36}));
    auto limit = std::chrono::steady_clock::now() + 5s;
    while (router.pendingCount() && std::chrono::steady_clock::now() < limit) {
        router.poll();
        std::this_thread::sleep_for(1ms);
    }
    auto &out = router.view().route(1);
    CHECK(router.pendingCount() == 0 && out.status == Status::Valid && (out.points.front() == Point{40, 36}));
}

void boundaryCases() {
    Config cfg;
    cfg.fastPaths = false;
    cfg.buffer = 8;

    // 1. Tight corridor border-hugging detour & wide corridor channel centering
    {
        LocalOrthogonalRouter r(cfg);
        r.addBlock(block(1, {0, 0, 40, 40}));
        r.addBlock(block(2, {200, 0, 240, 40}));
        r.addBlock(block(3, {80, -20, 120, 60}));
        r.addBlock(block(4, {60, -100, 140, -36}));
        r.addConnector({1, 1, 1, 2, 2});

        // Wide channel centering pair (forced through horizontal corridor [268, 312] -> midline y=290)
        r.addBlock(block(10, {0, 200, 40, 240}));
        r.addBlock(block(11, {240, 320, 280, 360}));
        r.addBlock(block(12, {90, 120, 220, 260}));
        r.addBlock(block(13, {0, 320, 180, 380}));
        r.addConnector({2, 10, 1, 11, 2});

        r.routeAllParallel(1);
        CHECK(r.route(1).status == Status::Valid);
        CHECK(r.route(2).status == Status::Valid);
        bool passedCorridorCenter = false;
        for (const auto &pt : r.route(2).points)
            if (std::abs(pt.y - 290.0) < 1.0)
                passedCorridorCenter = true;
        CHECK(passedCorridorCenter);
    }

    // 2. Multi-line fan-out sharing the same pin
    {
        LocalOrthogonalRouter r(cfg);
        r.addBlock({1, {0, 100, 40, 140}, {{1, 1, .5, Right}}});
        for (Id i = 0; i < 4; ++i) {
            double y = 20.0 + i * 70.0;
            r.addBlock(block(10 + i, {220, y, 260, y + 40}));
            r.addConnector({100 + i, 1, 1, 10 + i, 2});
        }
        r.addBlock(block(50, {110, 80, 150, 160}));
        r.routeAllParallel(1);
        for (Id i = 0; i < 4; ++i)
            CHECK(r.route(100 + i).status == Status::Valid);
    }

    // 3. Half-enclosed U-shaped pocket around start and end
    {
        LocalOrthogonalRouter r(cfg);
        r.addBlock(block(1, {0, 0, 40, 40}));
        r.addBlock(block(2, {260, 0, 300, 40}));
        r.addBlock(block(3, {70, -60, 100, 60}));
        r.addBlock(block(4, {-20, -60, 65, -25}));
        r.addBlock(block(5, {200, -10, 230, 100}));
        r.addBlock(block(6, {235, 65, 320, 100}));
        r.addConnector({1, 1, 1, 2, 2});
        r.routeAllParallel(1);
        CHECK(r.route(1).status == Status::Valid);
    }
}

int main(int argc, char **argv) {
    const std::pair<std::string, void (*)()> cases[] = {{"SpatialGridBasic", grid},
                                                        {"DirtyOnlyNoRerouteOnOverlap", dirty},
                                                        {"LocalObstacleAvoidance", avoid},
                                                        {"AdaptiveWindowExpansion", expansion},
                                                        {"OverlappingBlocksFallback", overlap},
                                                        {"ParallelColdStartDeterminism", parallel},
                                                        {"FastReuseAndThirdPartyObstruction", fastAndObstruction},
                                                        {"AsyncDiscardsStaleResults", asyncRouting},
                                                        {"LazyHananAStarBoundaryCases", boundaryCases}};
    try {
        for (auto &t : cases)
            if (argc == 1 || t.first == argv[1]) {
                t.second();
                std::cout << t.first << " PASS\n";
                if (argc > 1)
                    return 0;
            }
        return argc == 1 ? 0 : 2;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
