#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "localroute/local_route_solver.h"

namespace localroute {
namespace {

using Clock = std::chrono::steady_clock;

constexpr std::uint8_t kBlkOther = 1;
constexpr std::uint8_t kBlkSrc = 2;
constexpr std::uint8_t kBlkDst = 4;
constexpr std::uint8_t kBlkAny = kBlkOther | kBlkSrc | kBlkDst;
constexpr std::uint8_t kEdgeBorder = 8;

constexpr int kDirDx[4] = {0, 0, -1, 1};
constexpr int kDirDy[4] = {-1, 1, 0, 0};
constexpr unsigned kDirMask[4] = {kUp, kDown, kLeft, kRight};
constexpr int kOppIdx[4] = {1, 0, 3, 2};

Point PinPosition(Rect a, const Pin& p) {
  return {p.x == 1.0 ? a.r : a.l + p.x * (a.r - a.l),
          p.y == 1.0 ? a.b : a.t + p.y * (a.b - a.t)};
}

const Pin* FindPin(const Block& b, Id id) {
  for (const auto& p : b.pins) {
    if (p.id == id) {
      return &p;
    }
  }
  return nullptr;
}

bool Crosses(Point a, Point b, Rect r) {
  const Rect s = Box(a, b);
  return s.r > r.l && s.l < r.r && s.b > r.t && s.t < r.b;
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

void DedupPreservePins(std::vector<double>* v, double pin_a, double pin_b) {
  std::sort(v->begin(), v->end());
  if (v->empty()) {
    return;
  }
  constexpr double kEps = 1e-9;
  size_t write = 0;
  for (size_t i = 0; i < v->size(); ++i) {
    const double x = (*v)[i];
    if (write == 0) {
      (*v)[write++] = x;
      continue;
    }
    double& prev = (*v)[write - 1];
    if (std::abs(x - prev) <= kEps) {
      const bool x_is_pin = (x == pin_a || x == pin_b);
      const bool prev_is_pin = (prev == pin_a || prev == pin_b);
      if (x_is_pin && !prev_is_pin) {
        prev = x;
      } else if (x_is_pin && prev_is_pin && x != prev) {
        (*v)[write++] = x;
      }
    } else {
      (*v)[write++] = x;
    }
  }
  v->resize(write);
}

void InjectChannelMidlines(std::vector<double>* v, double buffer) {
  if (v->size() >= 200 || v->size() < 2) {
    return;
  }
  const double min_gap = std::max(buffer * 2.5, 16.0);
  constexpr size_t kMaxCoords = 240;
  std::vector<std::pair<double, double>> gaps;
  gaps.reserve(v->size());
  for (size_t i = 0; i + 1 < v->size(); ++i) {
    const double w = (*v)[i + 1] - (*v)[i];
    if (w > min_gap) {
      gaps.emplace_back(w, 0.5 * ((*v)[i] + (*v)[i + 1]));
    }
  }
  if (gaps.empty()) {
    return;
  }
  const size_t budget = kMaxCoords - v->size();
  if (gaps.size() > budget) {
    std::nth_element(
        gaps.begin(), gaps.begin() + static_cast<std::ptrdiff_t>(budget),
        gaps.end(),
        [](const auto& a, const auto& b) { return a.first > b.first; });
    gaps.resize(budget);
  }
  for (const auto& g : gaps) {
    v->push_back(g.second);
  }
  std::sort(v->begin(), v->end());
}

int FindCoordIndex(const std::vector<double>& v, double target) {
  const auto it = std::lower_bound(v.begin(), v.end(), target);
  if (it != v.end() && *it == target) {
    return static_cast<int>(it - v.begin());
  }
  if (it != v.begin() && std::abs(*(it - 1) - target) <= 1e-12) {
    return static_cast<int>((it - 1) - v.begin());
  }
  if (it != v.end() && std::abs(*it - target) <= 1e-12) {
    return static_cast<int>(it - v.begin());
  }
  return -1;
}

int MinBendsToEnterRight(double dx, double dy, int cur_dir) {
  if (cur_dir == 3) {
    if (dx > 0.0) {
      return dy == 0.0 ? 0 : 2;
    }
    return 4;
  }
  if (cur_dir == 2) {
    return dy == 0.0 ? 4 : 2;
  }
  const bool y_ok =
      (dy == 0.0) || (dy > 0.0 && cur_dir == 1) || (dy < 0.0 && cur_dir == 0);
  return (dx > 0.0 && y_ok) ? 1 : 3;
}

int MinRemainingBends(double x, double y, Point b, int dir_idx,
                      unsigned dp_dirs) {
  const double dx = b.x - x;
  const double dy = b.y - y;
  if (dx == 0.0 && dy == 0.0) {
    return 0;
  }
  // Rotate frame so each allowed entry direction maps to +X (Right = 3).
  int best = 4;
  for (int d_out = 0; d_out < 4; ++d_out) {
    if ((kDirMask[d_out] & dp_dirs) == 0) {
      continue;
    }
    const int d_enter = kOppIdx[d_out];
    int bends = 4;
    if (d_enter == 3) {
      bends = MinBendsToEnterRight(dx, dy, dir_idx);
    } else if (d_enter == 2) {
      bends = MinBendsToEnterRight(-dx, -dy, kOppIdx[dir_idx]);
    } else if (d_enter == 1) {
      constexpr int kRotCw[4] = {2, 3, 1, 0};
      bends = MinBendsToEnterRight(dy, -dx, kRotCw[dir_idx]);
    } else {
      constexpr int kRotCcw[4] = {3, 2, 0, 1};
      bends = MinBendsToEnterRight(-dy, dx, kRotCcw[dir_idx]);
    }
    if (bends < best) {
      best = bends;
    }
  }
  return best;
}

struct HeapNode {
  double f = 0.0;
  double g = 0.0;
  std::int32_t state = 0;
};

struct HeapCmp {
  bool operator()(const HeapNode& a, const HeapNode& b) const {
    if (a.f != b.f) {
      return a.f > b.f;
    }
    // Break exact f-ties toward goal (larger g implies smaller h).
    return a.g < b.g;
  }
};

struct EntryTarget {
  int ix = 0;
  int iy = 0;
  int d_enter_idx = 0;
  int d_out_idx = 0;
  double cost = 0.0;
};

struct SolverWorkspace {
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<std::uint8_t> edge_h;
  std::vector<std::uint8_t> edge_v;
  std::vector<double> g_cost;
  std::vector<std::int32_t> parent;
  std::vector<std::uint32_t> visited_stamp;
  std::vector<std::uint32_t> closed_stamp;
  std::vector<HeapNode> heap;
  std::vector<Point> raw_pts;
  std::uint32_t run_stamp = 1;

  void PrepareStates(size_t num_states) {
    if (g_cost.size() < num_states) {
      g_cost.resize(num_states);
      parent.resize(num_states);
      visited_stamp.resize(num_states, 0);
      closed_stamp.resize(num_states, 0);
    }
    if (run_stamp == std::numeric_limits<std::uint32_t>::max()) {
      std::fill(visited_stamp.begin(), visited_stamp.end(), 0);
      std::fill(closed_stamp.begin(), closed_stamp.end(), 0);
      run_stamp = 1;
    } else {
      ++run_stamp;
    }
    heap.clear();
  }
};

class LazyHananAStarSolverImpl final : public LocalRouteSolver {
 public:
  const char* name() const override { return "LazyHananAStarSolver"; }

  std::vector<Point> Solve(const Request& q) const override {
    Clock::time_point t_start = q.profile ? Clock::now() : Clock::time_point();

    const Block* src = nullptr;
    const Block* dst = nullptr;
    for (const Block* b_obs : q.obstacles) {
      if (b_obs->id == q.conn.src) {
        src = b_obs;
      }
      if (b_obs->id == q.conn.dst) {
        dst = b_obs;
      }
    }
    if (!src || !dst) {
      return {};
    }
    const Pin* sp = FindPin(*src, q.conn.sp);
    const Pin* dp = FindPin(*dst, q.conn.dp);
    if (!sp || !dp) {
      return {};
    }

    const Point a = PinPosition(src->rect, *sp);
    const Point b = PinPosition(dst->rect, *dp);
    if (a == b) {
      return {};
    }

    const Rect aa = src->rect.Pad(q.cfg.buffer);
    const Rect bb = dst->rect.Pad(q.cfg.buffer);

    if (a.x == b.x || a.y == b.y) {
      const unsigned d = OrthoDir(a, b);
      const int d_idx = (d == kUp ? 0 : d == kDown ? 1 : d == kLeft ? 2 : 3);
      const unsigned d_opp = kDirMask[kOppIdx[d_idx]];
      if ((d & sp->dirs) && (d_opp & dp->dirs)) {
        bool ok = !aa.Inside(b) &&
                  (src->rect.Inside(a) || !Crosses(a, b, src->rect)) &&
                  !bb.Inside(a) &&
                  (dst->rect.Inside(b) || !Crosses(b, a, dst->rect));
        if (ok) {
          for (const Block* b_obs : q.obstacles) {
            if (b_obs->id == q.conn.src || b_obs->id == q.conn.dst) {
              continue;
            }
            if (Crosses(a, b, b_obs->rect.Pad(q.cfg.buffer))) {
              ok = false;
              break;
            }
          }
        }
        if (ok) {
          if (q.profile) {
            q.profile->graph_ms += std::chrono::duration<double, std::milli>(
                                       Clock::now() - t_start)
                                       .count();
          }
          return {a, b};
        }
      }
    }

    thread_local SolverWorkspace ws;
    auto& xs = ws.xs;
    auto& ys = ws.ys;
    xs.clear();
    ys.clear();
    xs.reserve(q.obstacles.size() * 2 + 8);
    ys.reserve(q.obstacles.size() * 2 + 8);

    xs.push_back(a.x);
    xs.push_back(b.x);
    ys.push_back(a.y);
    ys.push_back(b.y);

    double min_x = std::min(a.x, b.x);
    double max_x = std::max(a.x, b.x);
    double min_y = std::min(a.y, b.y);
    double max_y = std::max(a.y, b.y);
    for (const Block* b_obs : q.obstacles) {
      const Rect r = b_obs->rect.Pad(q.cfg.buffer);
      xs.push_back(r.l);
      xs.push_back(r.r);
      ys.push_back(r.t);
      ys.push_back(r.b);
      min_x = std::min(min_x, r.l);
      max_x = std::max(max_x, r.r);
      min_y = std::min(min_y, r.t);
      max_y = std::max(max_y, r.b);
    }

    const double pad_out = std::max(q.cfg.buffer * 2.0, 32.0);
    xs.push_back(min_x - pad_out);
    xs.push_back(max_x + pad_out);
    ys.push_back(min_y - pad_out);
    ys.push_back(max_y + pad_out);

    DedupPreservePins(&xs, a.x, b.x);
    DedupPreservePins(&ys, a.y, b.y);
    InjectChannelMidlines(&xs, q.cfg.buffer);
    InjectChannelMidlines(&ys, q.cfg.buffer);

    const int w = static_cast<int>(xs.size());
    const int h = static_cast<int>(ys.size());
    if (w < 2 || h < 2) {
      return {};
    }

    const int sx = FindCoordIndex(xs, a.x);
    const int sy = FindCoordIndex(ys, a.y);
    const int gx = FindCoordIndex(xs, b.x);
    const int gy = FindCoordIndex(ys, b.y);
    if (sx < 0 || sy < 0 || gx < 0 || gy < 0) {
      return {};
    }

    auto& edge_h = ws.edge_h;
    auto& edge_v = ws.edge_v;
    edge_h.assign(static_cast<size_t>(h) * static_cast<size_t>(w - 1), 0);
    edge_v.assign(static_cast<size_t>(h - 1) * static_cast<size_t>(w), 0);

    constexpr double kBorderEps = 1e-9;
    for (const Block* b_obs : q.obstacles) {
      std::uint8_t bit = 0;
      if (b_obs->id == q.conn.src) {
        bit |= kBlkSrc;
      }
      if (b_obs->id == q.conn.dst) {
        bit |= kBlkDst;
      }
      if (b_obs->id != q.conn.src && b_obs->id != q.conn.dst) {
        bit |= kBlkOther;
      }

      const Rect r = b_obs->rect.Pad(q.cfg.buffer);

      const int hx0 = std::max(
          0, static_cast<int>(std::lower_bound(xs.begin(), xs.end(), r.l) -
                              xs.begin()) -
                 1);
      const int hx1 = std::min(
          w - 1, static_cast<int>(std::upper_bound(xs.begin(), xs.end(), r.r) -
                                  xs.begin()));
      const int hy0 = static_cast<int>(
          std::upper_bound(ys.begin(), ys.end(), r.t) - ys.begin());
      const int hy1 = static_cast<int>(
          std::lower_bound(ys.begin(), ys.end(), r.b) - ys.begin());
      for (int iy = hy0; iy < hy1; ++iy) {
        if (!(r.t < ys[iy] && ys[iy] < r.b)) {
          continue;
        }
        std::uint8_t* row_ptr = edge_h.data() + static_cast<size_t>(iy) *
                                                    static_cast<size_t>(w - 1);
        for (int ix = hx0; ix < hx1; ++ix) {
          if (xs[ix + 1] > r.l && xs[ix] < r.r) {
            row_ptr[ix] |= bit;
          }
        }
      }

      for (double border_y : {r.t, r.b}) {
        const int bt0 = std::max(
            0, static_cast<int>(std::lower_bound(ys.begin(), ys.end(),
                                                 border_y - kBorderEps) -
                                ys.begin()));
        const int bt1 = std::min(
            h, static_cast<int>(std::upper_bound(ys.begin(), ys.end(),
                                                 border_y + kBorderEps) -
                                ys.begin()));
        for (int iy = bt0; iy < bt1; ++iy) {
          std::uint8_t* row_ptr =
              edge_h.data() +
              static_cast<size_t>(iy) * static_cast<size_t>(w - 1);
          for (int ix = hx0; ix < hx1; ++ix) {
            if (xs[ix + 1] > r.l && xs[ix] < r.r) {
              row_ptr[ix] |= kEdgeBorder;
            }
          }
        }
      }

      const int vy0 = std::max(
          0, static_cast<int>(std::lower_bound(ys.begin(), ys.end(), r.t) -
                              ys.begin()) -
                 1);
      const int vy1 = std::min(
          h - 1, static_cast<int>(std::upper_bound(ys.begin(), ys.end(), r.b) -
                                  ys.begin()));
      const int vx0 = static_cast<int>(
          std::upper_bound(xs.begin(), xs.end(), r.l) - xs.begin());
      const int vx1 = static_cast<int>(
          std::lower_bound(xs.begin(), xs.end(), r.r) - xs.begin());
      for (int iy = vy0; iy < vy1; ++iy) {
        if (!(ys[iy + 1] > r.t && ys[iy] < r.b)) {
          continue;
        }
        std::uint8_t* row_ptr =
            edge_v.data() + static_cast<size_t>(iy) * static_cast<size_t>(w);
        for (int ix = vx0; ix < vx1; ++ix) {
          if (r.l < xs[ix] && xs[ix] < r.r) {
            row_ptr[ix] |= bit;
          }
        }
      }

      for (double border_x : {r.l, r.r}) {
        const int bx0 = std::max(
            0, static_cast<int>(std::lower_bound(xs.begin(), xs.end(),
                                                 border_x - kBorderEps) -
                                xs.begin()));
        const int bx1 = std::min(
            w, static_cast<int>(std::upper_bound(xs.begin(), xs.end(),
                                                 border_x + kBorderEps) -
                                xs.begin()));
        for (int iy = vy0; iy < vy1; ++iy) {
          if (!(ys[iy + 1] > r.t && ys[iy] < r.b)) {
            continue;
          }
          std::uint8_t* row_ptr =
              edge_v.data() + static_cast<size_t>(iy) * static_cast<size_t>(w);
          for (int ix = bx0; ix < bx1; ++ix) {
            row_ptr[ix] |= kEdgeBorder;
          }
        }
      }
    }

    auto edge_flags = [&](int x0, int y0, int d_idx) -> std::uint8_t {
      if (d_idx == 0) {
        return edge_v[static_cast<size_t>(y0 - 1) * static_cast<size_t>(w) +
                      static_cast<size_t>(x0)];
      }
      if (d_idx == 1) {
        return edge_v[static_cast<size_t>(y0) * static_cast<size_t>(w) +
                      static_cast<size_t>(x0)];
      }
      if (d_idx == 2) {
        return edge_h[static_cast<size_t>(y0) * static_cast<size_t>(w - 1) +
                      static_cast<size_t>(x0 - 1)];
      }
      return edge_h[static_cast<size_t>(y0) * static_cast<size_t>(w - 1) +
                    static_cast<size_t>(x0)];
    };

    auto step_length = [&](int x0, int y0, int x1, int y1) -> double {
      return std::abs(xs[x1] - xs[x0]) + std::abs(ys[y1] - ys[y0]);
    };

    EntryTarget entries[4];
    int num_entries = 0;
    for (int d_out_idx = 0; d_out_idx < 4; ++d_out_idx) {
      if ((kDirMask[d_out_idx] & dp->dirs) == 0) {
        continue;
      }
      int cx = gx;
      int cy = gy;
      double cost = 0.0;
      int steps = 0;
      for (;;) {
        const int nx = cx + kDirDx[d_out_idx];
        const int ny = cy + kDirDy[d_out_idx];
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
          break;
        }
        if (steps == 0 && !dst->rect.Inside(b) &&
            Crosses({xs[cx], ys[cy]}, {xs[nx], ys[ny]}, dst->rect)) {
          break;
        }
        const std::uint8_t f = edge_flags(cx, cy, d_out_idx);
        if (f & (kBlkOther | kBlkSrc)) {
          break;
        }
        cost += step_length(cx, cy, nx, ny);
        cx = nx;
        cy = ny;
        ++steps;
        if (!bb.Inside({xs[cx], ys[cy]})) {
          entries[num_entries++] = {cx, cy, kOppIdx[d_out_idx], d_out_idx,
                                    cost};
          break;
        }
      }
    }
    if (num_entries == 0) {
      if (q.profile) {
        q.profile->graph_ms +=
            std::chrono::duration<double, std::milli>(Clock::now() - t_start)
                .count();
      }
      return {};
    }

    const size_t num_states =
        static_cast<size_t>(w) * static_cast<size_t>(h) * 4;
    auto& heap = ws.heap;
    HeapCmp cmp;

    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double ab_len = std::hypot(abx, aby);
    const double line_nx = ab_len > 1.0 ? -aby / ab_len : 0.0;
    const double line_ny = ab_len > 1.0 ? abx / ab_len : 0.0;
    const bool truncated_window = q.obstacles.size() >= q.cfg.budget;
    auto perp_dist = [&](double x, double y) -> double {
      return ab_len > 1.0 ? std::abs((x - a.x) * line_nx + (y - a.y) * line_ny)
                          : 0.0;
    };

    auto run_a_star = [&](double heuristic_weight, double corridor_weight,
                          unsigned max_pops,
                          std::int32_t* best_goal_parent) -> bool {
      ws.PrepareStates(num_states);
      const std::uint32_t stamp = ws.run_stamp;

      auto push_start = [&](std::int32_t s, double g, double f) {
        if (ws.visited_stamp[s] != stamp || g < ws.g_cost[s]) {
          ws.visited_stamp[s] = stamp;
          ws.g_cost[s] = g;
          ws.parent[s] = -1;
          heap.push_back({f, g, s});
          std::push_heap(heap.begin(), heap.end(), cmp);
        }
      };

      for (int d0 = 0; d0 < 4; ++d0) {
        if ((kDirMask[d0] & sp->dirs) == 0) {
          continue;
        }
        int cx = sx;
        int cy = sy;
        double cost = 0.0;
        int steps = 0;
        for (;;) {
          const int nx = cx + kDirDx[d0];
          const int ny = cy + kDirDy[d0];
          if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
            break;
          }
          if (steps == 0 && !src->rect.Inside(a) &&
              Crosses({xs[cx], ys[cy]}, {xs[nx], ys[ny]}, src->rect)) {
            break;
          }
          const std::uint8_t f = edge_flags(cx, cy, d0);
          if (f & (kBlkOther | kBlkDst)) {
            break;
          }
          cost += step_length(cx, cy, nx, ny);
          cx = nx;
          cy = ny;
          ++steps;
          if (!aa.Inside({xs[cx], ys[cy]})) {
            const std::int32_t s0 = ((cy * w + cx) << 2) | d0;
            const double heuristic =
                std::abs(xs[cx] - b.x) + std::abs(ys[cy] - b.y) +
                MinRemainingBends(xs[cx], ys[cy], b, d0, dp->dirs) *
                    q.cfg.penalty;
            const double c_bias =
                corridor_weight > 0.0
                    ? corridor_weight * perp_dist(xs[cx], ys[cy])
                    : 0.0;
            push_start(s0, cost, cost + heuristic_weight * heuristic + c_bias);
            break;
          }
        }
      }

      if (heap.empty()) {
        return false;
      }

      double best_goal_cost = std::numeric_limits<double>::infinity();
      *best_goal_parent = -1;
      unsigned pops = 0;

      while (!heap.empty()) {
        const HeapNode cur = heap.front();
        std::pop_heap(heap.begin(), heap.end(), cmp);
        heap.pop_back();

        if (cur.state == -2) {
          if (cur.g <= best_goal_cost) {
            return *best_goal_parent != -1;
          }
          continue;
        }

        if (ws.closed_stamp[cur.state] == stamp ||
            cur.g > ws.g_cost[cur.state]) {
          continue;
        }
        ws.closed_stamp[cur.state] = stamp;

        if (++pops > max_pops) {
          return false;
        }

        const int cell = cur.state >> 2;
        const int cur_dir = cur.state & 3;
        const int iy = cell / w;
        const int ix = cell - iy * w;

        for (int ei = 0; ei < num_entries; ++ei) {
          const auto& e = entries[ei];
          if (e.ix != ix || e.iy != iy || cur_dir == e.d_out_idx) {
            continue;
          }
          const double cand_g =
              cur.g + e.cost + (cur_dir != e.d_enter_idx ? q.cfg.penalty : 0.0);
          if (cand_g < best_goal_cost) {
            best_goal_cost = cand_g;
            *best_goal_parent = cur.state;
            heap.push_back({cand_g, cand_g, -2});
            std::push_heap(heap.begin(), heap.end(), cmp);
          }
        }

        for (int d = 0; d < 4; ++d) {
          if (d == kOppIdx[cur_dir]) {
            continue;
          }
          const int nx = ix + kDirDx[d];
          const int ny = iy + kDirDy[d];
          if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
            continue;
          }
          const std::uint8_t f = edge_flags(ix, iy, d);
          if (f & kBlkAny) {
            continue;
          }

          const std::int32_t ns = ((ny * w + nx) << 2) | d;
          if (ws.closed_stamp[ns] == stamp) {
            continue;
          }

          const double ng = cur.g + step_length(ix, iy, nx, ny) +
                            (d != cur_dir ? q.cfg.penalty : 0.0) +
                            ((f & kEdgeBorder) ? 1e-4 : 0.0);
          if (ws.visited_stamp[ns] != stamp || ng < ws.g_cost[ns]) {
            ws.visited_stamp[ns] = stamp;
            ws.g_cost[ns] = ng;
            ws.parent[ns] = cur.state;
            const double heuristic =
                std::abs(xs[nx] - b.x) + std::abs(ys[ny] - b.y) +
                MinRemainingBends(xs[nx], ys[ny], b, d, dp->dirs) *
                    q.cfg.penalty;
            const double c_bias =
                corridor_weight > 0.0
                    ? corridor_weight * perp_dist(xs[nx], ys[ny])
                    : 0.0;
            heap.push_back(
                {ng + heuristic_weight * heuristic + c_bias, ng, ns});
            std::push_heap(heap.begin(), heap.end(), cmp);
          }
        }
      }
      return false;
    };

    if (q.profile) {
      const auto t_mid = Clock::now();
      q.profile->graph_ms +=
          std::chrono::duration<double, std::milli>(t_mid - t_start).count();
      t_start = t_mid;
    }

    std::int32_t best_goal_parent = -1;
    // Pass 1: exact admissible A* with tight cap (preserves channel centering).
    bool goal_found = run_a_star(1.0, 0.0, truncated_window ? 800u : 2500u,
                                 &best_goal_parent);
    // Pass 2: gentle corridor-guided weighted A* fallback on same grid.
    if (!goal_found && !heap.empty()) {
      goal_found = run_a_star(1.15, truncated_window ? 0.02 : 0.0, 20000u,
                              &best_goal_parent);
    }

    std::vector<Point> out;
    if (goal_found && best_goal_parent != -1) {
      auto& rev = ws.raw_pts;
      rev.clear();
      for (std::int32_t s = best_goal_parent; s != -1; s = ws.parent[s]) {
        const int cell = s >> 2;
        const int iy = cell / w;
        const int ix = cell - iy * w;
        rev.push_back({xs[ix], ys[iy]});
      }
      out.reserve(rev.size() + 2);
      out.push_back(a);
      for (auto it = rev.rbegin(); it != rev.rend(); ++it) {
        if (!(*it == out.back())) {
          out.push_back(*it);
        }
      }
      if (!(out.back() == b)) {
        out.push_back(b);
      }

      size_t count = 0;
      for (const Point p : out) {
        if (count >= 1 && p == out[count - 1]) {
          continue;
        }
        if (count >= 2 && OrthoDir(out[count - 2], out[count - 1]) ==
                              OrthoDir(out[count - 1], p)) {
          --count;
        }
        out[count++] = p;
      }
      out.resize(count);
    }

    if (q.profile) {
      q.profile->search_ms +=
          std::chrono::duration<double, std::milli>(Clock::now() - t_start)
              .count();
    }
    return out;
  }
};

}  // namespace

std::shared_ptr<const LocalRouteSolver> LazyHananAStarSolver() {
  return std::make_shared<LazyHananAStarSolverImpl>();
}

}  // namespace localroute
