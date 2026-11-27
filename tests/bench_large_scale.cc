#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "localroute/async_local_router.h"
#include "topology_generator.h"

namespace localroute {
namespace {

using Clock = std::chrono::steady_clock;

double ElapsedMs(Clock::time_point t) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

void PrintRow(const std::string& mode, unsigned n, const std::string& scene,
              unsigned threads, std::vector<double> v,
              const std::vector<const Route*>& routes,
              unsigned unexpected = 0) {
  unsigned valid = 0;
  unsigned display = 0;
  unsigned failed = 0;
  unsigned calls = 0;
  unsigned retries = 0;
  unsigned peak = 0;
  std::vector<unsigned> windows;
  bool known = true;
  Profile total;
  unsigned reused = 0;
  unsigned simple = 0;
  unsigned pending = 0;
  std::array<unsigned, 6> reasons{};
  for (const Route* r : routes) {
    if (!r) {
      known = false;
      break;
    }
    reused += r->method == RouteMethod::kReuse && r->status == Status::kValid;
    simple += r->method == RouteMethod::kSimple && r->status == Status::kValid;
    pending += r->status == Status::kPending;
    ++reasons[static_cast<unsigned>(r->outcome)];
    const auto& p = r->profile;
    total.query_ms += p.query_ms;
    total.setup_ms += p.setup_ms;
    total.graph_ms += p.graph_ms;
    total.search_ms += p.search_ms;
    total.validation_ms += p.validation_ms;
    total.native_other_ms += p.native_other_ms;
    total.invalid_attempts += p.invalid_attempts;
    total.collision_attempts += p.collision_attempts;
    total.capped_attempts += p.capped_attempts;
    valid += r->status == Status::kValid;
    display += r->status == Status::kDisplayOnly;
    failed += r->status == Status::kFailed;
    calls += r->calls;
    retries += r->retries;
    peak = std::max(peak, r->window);
    windows.push_back(r->window);
  }
  std::sort(v.begin(), v.end());
  std::sort(windows.begin(), windows.end());
  auto percentile = [&](double p) {
    return v[static_cast<size_t>(std::ceil(p * v.size())) - 1];
  };
  std::cout << mode << ',' << n << ',' << scene << ',' << threads << ','
            << v.size() << ','
            << std::accumulate(v.begin(), v.end(), 0.0) / v.size() << ','
            << percentile(0.5) << ',' << percentile(0.95) << ','
            << percentile(0.99) << ',' << v.back() << ',';
  if (known) {
    std::cout << valid << ',' << display << ',' << failed << ',' << calls << ','
              << retries << ','
              << (windows.empty() ? 0
                                  : windows[static_cast<size_t>(std::ceil(
                                                0.99 * windows.size())) -
                                            1])
              << ',' << peak << ',' << unexpected << ',' << reused << ','
              << simple << ',' << pending << ',' << total.query_ms << ','
              << total.setup_ms << ',' << total.graph_ms << ','
              << total.search_ms << ',' << total.validation_ms << ','
              << total.native_other_ms << ',' << total.invalid_attempts << ','
              << total.collision_attempts << ',' << total.capped_attempts << ','
              << reasons[1] << ',' << reasons[2] << ',' << reasons[3] << ','
              << reasons[4] << ',' << reasons[5];
  } else {
    for (int i = 0; i < 25; ++i) {
      std::cout << (i ? ",NA" : "NA");
    }
  }
  std::cout << std::endl;
}

void RunAsyncBench(const Topology& top, unsigned n) {
  auto now = Clock::now();
  LocalOrthogonalRouter initial;
  for (const auto& b : top.blocks) {
    initial.AddBlock(b);
  }
  for (const auto& c : top.conns) {
    initial.AddConnector(c);
  }
  AsyncLocalRouter router(std::move(initial));
  router.Prioritize(top.blocks.front().rect.Pad(512));
  router.StartAll();
  std::vector<const Route*> routes;
  PrintRow("async_owner", n, "first_interactive_proxy", 1, {ElapsedMs(now)},
           routes);
  auto wait = [&] {
    const auto limit = Clock::now() + std::chrono::minutes(5);
    while (router.PendingCount()) {
      if (Clock::now() > limit) {
        throw std::runtime_error("async benchmark timeout");
      }
      if (!router.Poll()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  };
  wait();
  const double cold = ElapsedMs(now);
  for (const auto& c : top.conns) {
    routes.push_back(&router.view().route(c.id));
  }
  PrintRow("async_owner", n, "async_full", 1, {cold}, routes);
  for (unsigned count : {1u, 20u}) {
    std::vector<double> samples;
    for (unsigned frame = 0; frame < 100; ++frame) {
      std::vector<std::pair<Id, Rect>> moves;
      for (unsigned j = 0; j < count; ++j) {
        const Id id = count == 1 ? n / 2 : (j / 10) * top.side + j % 10;
        const Rect b = top.blocks[id].rect;
        const double d = static_cast<double>(frame % 10) * 2.0;
        moves.push_back({id, {b.l + d, b.t + d, b.r + d, b.b + d}});
      }
      now = Clock::now();
      router.Move(moves);
      router.Poll();
      samples.push_back(ElapsedMs(now));
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    now = Clock::now();
    router.Settle();
    wait();
    const double settled = ElapsedMs(now);
    PrintRow("async_owner", n, count == 1 ? "owner_A_proxy" : "owner_B20_proxy",
             1, samples, {});
    PrintRow("async_owner", n, count == 1 ? "settled_A" : "settled_B20", 1,
             {settled}, {});
  }
}

}  // namespace
}  // namespace localroute

int main(int argc, char** argv) {
  using namespace localroute;
  if (argc < 3) {
    return 2;
  }
  const std::string mode = argv[1];
  const unsigned n = std::stoul(argv[2]);
  if (n < 64 || n > 10000) {
    return 2;
  }
  const Topology top = GenerateTopology(n);
  if (mode == "async") {
    RunAsyncBench(top, n);
    return 0;
  }
  if (mode != "local1" && mode != "local4" && mode != "profile4" &&
      mode != "legacy4") {
    return 2;
  }
  const unsigned threads = mode == "local1" ? 1 : 4;
  auto start = Clock::now();
  Config cfg;
  cfg.profile = mode == "profile4" || mode == "legacy4";
  cfg.fast_paths = mode != "legacy4";
  LocalOrthogonalRouter r(cfg);
  std::vector<std::vector<Id>> adj(n);
  for (const auto& b : top.blocks) {
    r.AddBlock(b);
  }
  for (const auto& c : top.conns) {
    r.AddConnector(c);
    adj[c.src].push_back(c.id);
    adj[c.dst].push_back(c.id);
  }
  r.RouteAllParallel(threads);
  const double cold = ElapsedMs(start);
  std::vector<const Route*> all;
  for (const auto& c : top.conns) {
    all.push_back(&r.route(c.id));
  }
  PrintRow(r.solver_name(), n, "D_cold", threads, {cold}, all);
  if (cfg.profile && argc > 3) {
    std::ofstream out(std::string(argv[3]) + "/" + mode + "-" +
                      std::to_string(n) + "-lines.csv");
    if (!out) {
      throw std::runtime_error("cannot write line profile");
    }
    out << "id,span,window,status,method,outcome,calls,retries,query_ms,"
           "setup_ms,graph_ms,search_ms,validation_ms,other_ms,"
           "invalid_attempts,collision_attempts,capped_attempts\n";
    for (const auto& c : top.conns) {
      const Rect& a = r.block(c.src).rect;
      const Rect& b = r.block(c.dst).rect;
      const Route& q = r.route(c.id);
      const Profile& p = q.profile;
      out << c.id << ','
          << (std::abs(a.l + a.r - b.l - b.r) +
              std::abs(a.t + a.b - b.t - b.b)) *
                 0.5
          << ',' << q.window << ',' << static_cast<unsigned>(q.status) << ','
          << static_cast<unsigned>(q.method) << ','
          << static_cast<unsigned>(q.outcome) << ',' << q.calls << ','
          << q.retries << ',' << p.query_ms << ',' << p.setup_ms << ','
          << p.graph_ms << ',' << p.search_ms << ',' << p.validation_ms << ','
          << p.native_other_ms << ',' << p.invalid_attempts << ','
          << p.collision_attempts << ',' << p.capped_attempts << '\n';
    }
  }
  if (mode == "legacy4" || mode == "profile4") {
    return 0;
  }
  const Rect mid = r.block(n / 2).rect;
  std::vector<double> d_frames;
  std::vector<Route> d_stats;
  for (int i = 0; i < 10; ++i) {
    start = Clock::now();
    r.MoveBlock(n / 2, i % 2 ? 2 : -2, 0);
    const auto changed = r.RerouteAffected({n / 2});
    d_frames.push_back(ElapsedMs(start));
    for (Id id : changed) {
      d_stats.push_back(r.route(id));
    }
  }
  std::vector<const Route*> d_refs;
  for (const auto& q : d_stats) {
    d_refs.push_back(&q);
  }
  PrintRow(r.solver_name(), n, "D_drag", 1, d_frames, d_refs);
  if (threads == 1) {
    return 0;
  }
  r.MoveBlocksBatch({{n / 2, mid}});
  r.RerouteAffected({n / 2});
  std::vector<Id> selected;
  std::mt19937 rng(42);
  for (unsigned deg = 2; deg <= 8 && selected.size() < 10; ++deg) {
    for (unsigned offset = 0, base = rng() % n;
         offset < n && selected.size() < 10; ++offset) {
      const Id id = (base + offset) % n;
      if (adj[id].size() == deg) {
        selected.push_back(id);
        if (deg != 4 || selected.size() >= 5) {
          break;
        }
      }
    }
  }
  for (Id id = 0; id < n && selected.size() < 10; ++id) {
    if (adj[id].size() >= 2 && adj[id].size() <= 8 &&
        std::find(selected.begin(), selected.end(), id) == selected.end()) {
      selected.push_back(id);
    }
  }
  if (selected.size() != 10) {
    return 3;
  }
  auto run = [&](const std::string& scene, const std::vector<Id>& ids,
                 unsigned frames, std::vector<double>* times,
                 std::vector<Route>* stats, unsigned* unexpected) {
    std::vector<Rect> initial;
    std::set<Id> expected;
    for (Id id : ids) {
      initial.push_back(r.block(id).rect);
      expected.insert(adj[id].begin(), adj[id].end());
    }
    for (unsigned i = 0; i < frames; ++i) {
      const double dx =
          scene == "C" ? (1 + i) * 2.0 : static_cast<double>(i % 10) * 2.0;
      const double dy = scene == "C" ? 49.0 : static_cast<double>(i / 10) * 3.0;
      std::vector<std::pair<Id, Rect>> moves;
      for (size_t j = 0; j < ids.size(); ++j) {
        const Rect b = initial[j];
        moves.push_back({ids[j], {b.l + dx, b.t + dy, b.r + dx, b.b + dy}});
      }
      const auto tick = Clock::now();
      r.MoveBlocksBatch(moves);
      const auto changed = r.RerouteAffected(ids);
      times->push_back(ElapsedMs(tick));
      if (changed.size() != expected.size() ||
          !std::equal(changed.begin(), changed.end(), expected.begin())) {
        ++(*unexpected);
      }
      for (Id id : changed) {
        Route q = r.route(id);
        q.points.clear();
        stats->push_back(std::move(q));
      }
      if (scene == "C" && n == 10000 && i + 1 == frames && argc > 3) {
        const Rect v = initial.front().Pad(310);
        r.ExportRegionToSvg(std::string(argv[3]) + "/drag_after_lazy_astar.svg",
                            v, changed);
      }
    }
    std::vector<std::pair<Id, Rect>> restore;
    for (size_t j = 0; j < ids.size(); ++j) {
      restore.push_back({ids[j], initial[j]});
    }
    r.MoveBlocksBatch(restore);
    r.RerouteAffected(ids);
  };

  for (const std::string& scene : {"A", "B20", "B50", "C"}) {
    std::vector<double> times;
    std::vector<Route> stats;
    unsigned extra = 0;
    if (scene == "A") {
      for (Id id : selected) {
        run(scene, {id}, 20, &times, &stats, &extra);
      }
    } else if (scene == "C") {
      const Id id =
          static_cast<Id>(std::max_element(adj.begin(), adj.end(),
                                           [](const auto& a, const auto& b) {
                                             return a.size() < b.size();
                                           }) -
                          adj.begin());
      if (n == 10000 && argc > 3) {
        r.ExportRegionToSvg(std::string(argv[3]) + "/drag_before.svg",
                            r.block(id).rect.Pad(310));
      }
      run(scene, {id}, 200, &times, &stats, &extra);
    } else {
      const unsigned count = scene == "B20" ? 20 : 50;
      std::vector<Id> ids;
      for (Id i = 0; i < count; ++i) {
        ids.push_back((i / 10) * top.side + i % 10);
      }
      run(scene, ids, 200, &times, &stats, &extra);
    }
    std::vector<const Route*> refs;
    for (const auto& q : stats) {
      refs.push_back(&q);
    }
    PrintRow(r.solver_name(), n, scene, 1, times, refs, extra);
    if (extra) {
      return 1;
    }
  }
  return 0;
}
