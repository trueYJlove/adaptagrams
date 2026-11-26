#include "localroute/AsyncLocalRouter.h"
#include "topology_generator.h"
#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <set>
#include <thread>

using namespace localroute;
using Clock = std::chrono::steady_clock;
double ms(Clock::time_point t) { return std::chrono::duration<double, std::milli>(Clock::now() - t).count(); }

void row(const std::string &mode, unsigned n, const std::string &scene, unsigned threads,
         std::vector<double> v, const std::vector<const Route *> &routes, unsigned unexpected = 0) {
    unsigned valid = 0, display = 0, failed = 0, calls = 0, retries = 0, peak = 0;
    std::vector<unsigned> windows;
    bool known = true;
    Profile total;
    unsigned reused = 0, simple = 0, pending = 0;
    std::array<unsigned, 6> reasons{};
    for (auto r : routes) {
        if (!r) {
            known = false;
            break;
        }
        reused += r->method == RouteMethod::Reuse && r->status == Status::Valid;
        simple += r->method == RouteMethod::Simple && r->status == Status::Valid;
        pending += r->status == Status::Pending;
        ++reasons[unsigned(r->outcome)];
        auto &p = r->profile;
        total.queryMs += p.queryMs;
        total.setupMs += p.setupMs;
        total.graphMs += p.graphMs;
        total.searchMs += p.searchMs;
        total.validationMs += p.validationMs;
        total.nativeOtherMs += p.nativeOtherMs;
        total.invalidAttempts += p.invalidAttempts;
        total.collisionAttempts += p.collisionAttempts;
        total.cappedAttempts += p.cappedAttempts;
        valid += r->status == Status::Valid;
        display += r->status == Status::DisplayOnly;
        failed += r->status == Status::Failed;
        calls += r->calls;
        retries += r->retries;
        peak = std::max(peak, r->window);
        windows.push_back(r->window);
    }
    std::sort(v.begin(), v.end());
    std::sort(windows.begin(), windows.end());
    auto percentile = [&](double p) { return v[size_t(std::ceil(p * v.size())) - 1]; };
    std::cout << mode << ',' << n << ',' << scene << ',' << threads << ',' << v.size() << ','
              << std::accumulate(v.begin(), v.end(), 0.) / v.size() << ',' << percentile(.5) << ','
              << percentile(.95) << ',' << percentile(.99) << ',' << v.back() << ',';
    if (known)
        std::cout << valid << ',' << display << ',' << failed << ',' << calls << ',' << retries << ','
                  << (windows.empty() ? 0 : windows[size_t(std::ceil(.99 * windows.size())) - 1]) << ','
                  << peak << ',' << unexpected << ',' << reused << ',' << simple << ',' << pending << ','
                  << total.queryMs << ',' << total.setupMs << ',' << total.graphMs << ',' << total.searchMs << ','
                  << total.validationMs << ',' << total.nativeOtherMs << ',' << total.invalidAttempts << ','
                  << total.collisionAttempts << ',' << total.cappedAttempts << ',' << reasons[1] << ','
                  << reasons[2] << ',' << reasons[3] << ',' << reasons[4] << ',' << reasons[5];
    else
        for (int i = 0; i < 25; ++i)
            std::cout << (i ? ",NA" : "NA");
    std::cout << std::endl;
}

void asyncBench(const Topology &top, unsigned n) {
    auto now = Clock::now();
    LocalOrthogonalRouter initial;
    for (auto b : top.blocks)
        initial.addBlock(std::move(b));
    for (auto c : top.conns)
        initial.addConnector(c);
    AsyncLocalRouter router(std::move(initial));
    router.prioritize(top.blocks.front().rect.pad(512));
    router.startAll();
    std::vector<const Route *> routes;
    row("async_owner", n, "first_interactive_proxy", 1, {ms(now)}, routes);
    auto wait = [&] {
        auto limit = Clock::now() + std::chrono::minutes(5);
        while (router.pendingCount()) {
            if (Clock::now() > limit)
                throw std::runtime_error("async benchmark timeout");
            if (!router.poll())
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    };
    wait();
    double cold = ms(now);
    for (auto c : top.conns)
        routes.push_back(&router.view().route(c.id));
    row("async_owner", n, "async_full", 1, {cold}, routes);
    for (unsigned count : {1u, 20u}) {
        std::vector<double> samples;
        double settled;
        for (unsigned frame = 0; frame < 100; ++frame) {
            std::vector<std::pair<Id, Rect>> moves;
            for (unsigned j = 0; j < count; ++j) {
                Id id = count == 1 ? n / 2 : (j / 10) * top.side + j % 10;
                auto b = top.blocks[id].rect;
                double d = double(frame % 10) * 2;
                moves.push_back({id, {b.l + d, b.t + d, b.r + d, b.b + d}});
            }
            now = Clock::now();
            router.move(moves);
            router.poll();
            samples.push_back(ms(now));
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        now = Clock::now();
        router.settle();
        wait();
        settled = ms(now);
        row("async_owner", n, count == 1 ? "owner_A_proxy" : "owner_B20_proxy", 1, samples, {});
        row("async_owner", n, count == 1 ? "settled_A" : "settled_B20", 1, {settled}, {});
    }
}

int main(int argc, char **argv) {
    if (argc < 3)
        return 2;
    std::string mode = argv[1];
    unsigned n = std::stoul(argv[2]);
    if (n < 64 || n > 10000)
        return 2;
    auto top = generate(n);
    if (mode == "async") {
        asyncBench(top, n);
        return 0;
    }
    if (mode != "local1" && mode != "local4" && mode != "profile4" && mode != "legacy4")
        return 2;
    unsigned threads = mode == "local1" ? 1 : 4;
    auto start = Clock::now();
    Config cfg;
    cfg.profile = mode == "profile4" || mode == "legacy4";
    cfg.fastPaths = mode != "legacy4";
    LocalOrthogonalRouter r(cfg);
    std::vector<std::vector<Id>> adj(n);
    for (auto b : top.blocks)
        r.addBlock(b);
    for (auto c : top.conns) {
        r.addConnector(c);
        adj[c.src].push_back(c.id);
        adj[c.dst].push_back(c.id);
    }
    r.routeAllParallel(threads);
    double cold = ms(start);
    std::vector<const Route *> all;
    for (auto c : top.conns)
        all.push_back(&r.route(c.id));
    row(r.solverName(), n, "D_cold", threads, {cold}, all);
    if (cfg.profile && argc > 3) {
        std::ofstream out(std::string(argv[3]) + "/" + mode + "-" + std::to_string(n) + "-lines.csv");
        if (!out)
            throw std::runtime_error("cannot write line profile");
        out << "id,span,window,status,method,outcome,calls,retries,query_ms,setup_ms,graph_ms,search_ms,validation_ms,other_ms,invalid_attempts,collision_attempts,capped_attempts\n";
        for (auto c : top.conns) {
            auto &a = r.block(c.src).rect, &b = r.block(c.dst).rect;
            auto &q = r.route(c.id);
            auto &p = q.profile;
            out << c.id << ',' << (std::abs(a.l + a.r - b.l - b.r) + std::abs(a.t + a.b - b.t - b.b)) * .5
                << ',' << q.window << ',' << unsigned(q.status) << ',' << unsigned(q.method) << ','
                << unsigned(q.outcome) << ',' << q.calls << ',' << q.retries << ',' << p.queryMs << ','
                << p.setupMs << ',' << p.graphMs << ',' << p.searchMs << ',' << p.validationMs << ','
                << p.nativeOtherMs << ',' << p.invalidAttempts << ',' << p.collisionAttempts << ','
                << p.cappedAttempts << '\n';
        }
    }
    if (mode == "legacy4" || mode == "profile4")
        return 0;
    auto mid = r.block(n / 2).rect;
    std::vector<double> dFrames;
    std::vector<Route> dStats;
    for (int i = 0; i < 10; ++i) {
        start = Clock::now();
        r.moveBlock(n / 2, i % 2 ? 2 : -2, 0);
        auto changed = r.rerouteAffected({n / 2});
        dFrames.push_back(ms(start));
        for (auto id : changed)
            dStats.push_back(r.route(id));
    }
    std::vector<const Route *> dRefs;
    for (auto &q : dStats)
        dRefs.push_back(&q);
    row(r.solverName(), n, "D_drag", 1, dFrames, dRefs);
    if (threads == 1)
        return 0;
    r.moveBlocksBatch({{n / 2, mid}});
    r.rerouteAffected({n / 2});
    std::vector<Id> selected;
    std::mt19937 rng(42);
    for (unsigned deg = 2; deg <= 8 && selected.size() < 10; ++deg)
        for (unsigned offset = 0, base = rng() % n; offset < n && selected.size() < 10; ++offset) {
            auto id = (base + offset) % n;
            if (adj[id].size() == deg) {
                selected.push_back(id);
                if (deg != 4 || selected.size() >= 5)
                    break;
            }
        }
    for (Id id = 0; id < n && selected.size() < 10; ++id)
        if (adj[id].size() >= 2 && adj[id].size() <= 8 &&
            std::find(selected.begin(), selected.end(), id) == selected.end())
            selected.push_back(id);
    if (selected.size() != 10)
        return 3;
    auto run = [&](const std::string &scene, const std::vector<Id> &ids, unsigned frames,
                   std::vector<double> &times, std::vector<Route> &stats, unsigned &unexpected) {
        std::vector<Rect> initial;
        std::set<Id> expected;
        for (auto id : ids) {
            initial.push_back(r.block(id).rect);
            expected.insert(adj[id].begin(), adj[id].end());
        }
        for (unsigned i = 0; i < frames; ++i) {
            double dx = scene == "C" ? (1 + i) * 2.0 : double(i % 10) * 2,
                   dy = scene == "C" ? 49. : double(i / 10) * 3;
            std::vector<std::pair<Id, Rect>> moves;
            for (size_t j = 0; j < ids.size(); ++j) {
                auto b = initial[j];
                moves.push_back({ids[j], {b.l + dx, b.t + dy, b.r + dx, b.b + dy}});
            }
            auto tick = Clock::now();
            r.moveBlocksBatch(moves);
            auto changed = r.rerouteAffected(ids);
            times.push_back(ms(tick));
            if (!std::includes(changed.begin(), changed.end(), expected.begin(), expected.end()))
                ++unexpected;
            for (Id id : changed) {
                auto &src = r.route(id);
                Route q = src;
                q.points.clear();
                stats.push_back(q);
            }
            if (scene == "C" && n == 10000 && i + 1 == frames && argc > 3) {
                auto v = initial.front().pad(310);
                r.exportRegionToSVG(std::string(argv[3]) + "/drag_after_lazy_astar.svg", v, changed);
            }
        }
        std::vector<std::pair<Id, Rect>> restore;
        for (size_t j = 0; j < ids.size(); ++j)
            restore.push_back({ids[j], initial[j]});
        r.moveBlocksBatch(restore);
        r.rerouteAffected(ids);
    };
    for (std::string scene : {"A", "B20", "B50", "C"}) {
        std::vector<double> times;
        std::vector<Route> stats;
        unsigned extra = 0;
        if (scene == "A")
            for (Id id : selected)
                run(scene, {id}, 20, times, stats, extra);
        else if (scene == "C") {
            auto id = Id(std::max_element(adj.begin(), adj.end(),
                                          [](auto &a, auto &b) { return a.size() < b.size(); }) -
                         adj.begin());
            if (n == 10000 && argc > 3)
                r.exportRegionToSVG(std::string(argv[3]) + "/drag_before.svg", r.block(id).rect.pad(310));
            run(scene, {id}, 200, times, stats, extra);
        } else {
            unsigned count = scene == "B20" ? 20 : 50;
            std::vector<Id> ids;
            for (Id i = 0; i < count; ++i)
                ids.push_back((i / 10) * top.side + i % 10);
            run(scene, ids, 200, times, stats, extra);
        }
        std::vector<const Route *> refs;
        for (auto &q : stats)
            refs.push_back(&q);
        row(r.solverName(), n, scene, 1, times, refs, extra);
        if (extra)
            return 1;
    }
}
