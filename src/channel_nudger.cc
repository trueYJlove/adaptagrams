#include "channel_nudger.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <set>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "localroute/local_orthogonal_router.h"

namespace localroute {
namespace {

constexpr std::size_t kMaxLocalConns = 256;

enum class Axis { kVertical, kHorizontal };

struct NetKey {
  Id src_block = 0;
  Id src_pin = 0;

  bool operator==(const NetKey& other) const {
    return src_block == other.src_block && src_pin == other.src_pin;
  }

  bool operator<(const NetKey& other) const {
    return std::tie(src_block, src_pin) <
           std::tie(other.src_block, other.src_pin);
  }
};

struct MiddleSegment {
  Id conn_id = 0;
  std::size_t seg_idx = 0;
  NetKey net;
  double coord = 0.0;
  double min_k = 0.0;
  double max_k = 0.0;
  double prev_prim = 0.0;
  double next_prim = 0.0;
  double k_at_seg = 0.0;
  double k_at_next = 0.0;
};

struct TrackGroup {
  NetKey net;
  std::vector<const MiddleSegment*> segs;
  int shape_class = 0;
  double order_key = 0.0;
};

double PrimaryCoord(Point p, Axis axis) {
  return axis == Axis::kVertical ? p.x : p.y;
}

double CrossCoord(Point p, Axis axis) {
  return axis == Axis::kVertical ? p.y : p.x;
}

void SetPrimaryCoord(Point* p, Axis axis, double value) {
  if (axis == Axis::kVertical) {
    p->x = value;
  } else {
    p->y = value;
  }
}

Rect RouteBounds(const std::vector<Point>& pts) {
  Rect b{pts.front().x, pts.front().y, pts.front().x, pts.front().y};
  for (const Point p : pts) {
    b.l = std::min(b.l, p.x);
    b.r = std::max(b.r, p.x);
    b.t = std::min(b.t, p.y);
    b.b = std::max(b.b, p.y);
  }
  return b;
}

std::vector<Id> CollectParticipants(
    const Config& cfg, const SpatialGridIndex& index,
    const std::unordered_map<Id, Conn>& conns,
    const std::unordered_map<Id, std::vector<Id>>& adjacent,
    const std::unordered_map<Id, Route>& routes,
    const std::vector<Id>& seed_conns, bool full_graph) {
  std::set<Id> selected;
  if (full_graph) {
    for (const auto& kv : conns) {
      const auto rit = routes.find(kv.first);
      if (rit != routes.end() && rit->second.status == Status::kValid &&
          rit->second.points.size() >= 4) {
        selected.insert(kv.first);
      }
    }
    return {selected.begin(), selected.end()};
  }

  bool has_box = false;
  Rect union_box{0, 0, 0, 0};
  for (Id cid : seed_conns) {
    const auto rit = routes.find(cid);
    if (rit == routes.end() || rit->second.status != Status::kValid ||
        rit->second.points.size() < 4) {
      continue;
    }
    selected.insert(cid);
    const Rect rb = RouteBounds(rit->second.points);
    if (!has_box) {
      union_box = rb;
      has_box = true;
    } else {
      union_box.l = std::min(union_box.l, rb.l);
      union_box.t = std::min(union_box.t, rb.t);
      union_box.r = std::max(union_box.r, rb.r);
      union_box.b = std::max(union_box.b, rb.b);
    }
  }
  if (!has_box) {
    return {};
  }

  const double pad = std::max(cfg.nudge_distance * 2.0, 80.0);
  const std::vector<Id> nearby_blocks = index.Query(union_box.Pad(pad));
  for (Id bid : nearby_blocks) {
    if (selected.size() >= kMaxLocalConns) {
      break;
    }
    const auto ait = adjacent.find(bid);
    if (ait == adjacent.end()) {
      continue;
    }
    for (Id cid : ait->second) {
      if (selected.size() >= kMaxLocalConns) {
        break;
      }
      const auto rit = routes.find(cid);
      if (rit != routes.end() && rit->second.status == Status::kValid &&
          rit->second.points.size() >= 4) {
        selected.insert(cid);
      }
    }
  }
  return {selected.begin(), selected.end()};
}

std::vector<MiddleSegment> ExtractMiddleSegments(
    const std::vector<Id>& participants,
    const std::unordered_map<Id, Conn>& conns,
    const std::unordered_map<Id, Route>& routes, Axis axis) {
  std::vector<MiddleSegment> segments;
  for (Id cid : participants) {
    const auto rit = routes.find(cid);
    if (rit == routes.end() || rit->second.status != Status::kValid ||
        rit->second.points.size() < 4) {
      continue;
    }
    const auto& pts = rit->second.points;
    const Conn& conn = conns.at(cid);
    const NetKey net{conn.src, conn.sp};
    for (std::size_t j = 1; j + 2 < pts.size(); ++j) {
      const double p0_prim = PrimaryCoord(pts[j], axis);
      const double p1_prim = PrimaryCoord(pts[j + 1], axis);
      const double p0_k = CrossCoord(pts[j], axis);
      const double p1_k = CrossCoord(pts[j + 1], axis);
      if (std::abs(p0_prim - p1_prim) <= 1e-6 && std::abs(p0_k - p1_k) > 1e-3) {
        MiddleSegment seg;
        seg.conn_id = cid;
        seg.seg_idx = j;
        seg.net = net;
        seg.coord = 0.5 * (p0_prim + p1_prim);
        seg.min_k = std::min(p0_k, p1_k);
        seg.max_k = std::max(p0_k, p1_k);
        seg.prev_prim = PrimaryCoord(pts[j - 1], axis);
        seg.next_prim = PrimaryCoord(pts[j + 2], axis);
        seg.k_at_seg = p0_k;
        seg.k_at_next = p1_k;
        segments.push_back(seg);
      }
    }
  }
  return segments;
}

void ComputeOrderKey(TrackGroup* group) {
  const MiddleSegment* rep = group->segs.front();
  double best_span = rep->max_k - rep->min_k;
  for (const MiddleSegment* s : group->segs) {
    const double span = s->max_k - s->min_k;
    if (span > best_span + 1e-6) {
      best_span = span;
      rep = s;
    }
  }
  const bool in_neg = rep->prev_prim < rep->coord;
  const bool out_pos = rep->next_prim > rep->coord;
  const double a = rep->k_at_seg;
  const double b = rep->k_at_next;
  if (in_neg && out_pos) {
    group->shape_class = 0;
    group->order_key = (b > a) ? -a : a;
  } else if (!in_neg && !out_pos) {
    group->shape_class = 0;
    group->order_key = (b > a) ? a : -a;
  } else if (in_neg && !out_pos) {
    group->shape_class = 1;
    group->order_key = std::abs(b - a);
  } else {
    group->shape_class = 2;
    group->order_key = -std::abs(b - a);
  }
}

void NudgeAxis(
    const Config& cfg, const SpatialGridIndex& index,
    const std::unordered_map<Id, Block>& blocks,
    const std::unordered_map<Id, Conn>& conns,
    const std::vector<Id>& participants, Axis axis,
    std::unordered_map<Id, Route>* routes,
    const std::function<bool(const Conn&, std::vector<Point>*)>& validate_fn,
    const std::function<void(Id)>& on_route_updated,
    std::set<Id>* changed_out) {
  std::vector<MiddleSegment> all_segs =
      ExtractMiddleSegments(participants, conns, *routes, axis);
  if (all_segs.size() < 2) {
    return;
  }

  std::sort(all_segs.begin(), all_segs.end(),
            [](const MiddleSegment& a, const MiddleSegment& b) {
              if (a.coord != b.coord) {
                return a.coord < b.coord;
              }
              return std::tie(a.min_k, a.max_k, a.net, a.conn_id, a.seg_idx) <
                     std::tie(b.min_k, b.max_k, b.net, b.conn_id, b.seg_idx);
            });

  // Process coordinate bands separately without cross-bucket double counting.
  const double band_gap = std::max(1.5, cfg.nudge_distance * 0.8);
  for (std::size_t b_start = 0; b_start < all_segs.size();) {
    std::size_t b_end = b_start + 1;
    while (b_end < all_segs.size() &&
           all_segs[b_end].coord - all_segs[b_end - 1].coord <= band_gap) {
      ++b_end;
    }
    if (b_end - b_start < 2) {
      b_start = b_end;
      continue;
    }

    std::sort(all_segs.begin() + b_start, all_segs.begin() + b_end,
              [](const MiddleSegment& a, const MiddleSegment& b) {
                return std::tie(a.min_k, a.max_k, a.net, a.conn_id, a.seg_idx) <
                       std::tie(b.min_k, b.max_k, b.net, b.conn_id, b.seg_idx);
              });

    for (std::size_t c_start = b_start; c_start < b_end;) {
      double cluster_min_k = all_segs[c_start].min_k;
      double cluster_max_k = all_segs[c_start].max_k;
      std::size_t c_end = c_start + 1;
      while (c_end < b_end && all_segs[c_end].min_k < cluster_max_k - 1e-3) {
        cluster_min_k = std::min(cluster_min_k, all_segs[c_end].min_k);
        cluster_max_k = std::max(cluster_max_k, all_segs[c_end].max_k);
        ++c_end;
      }

      if (c_end - c_start < 2) {
        c_start = c_end;
        continue;
      }

      std::vector<TrackGroup> groups;
      double coord_sum = 0.0;
      for (std::size_t k = c_start; k < c_end; ++k) {
        const MiddleSegment* s = &all_segs[k];
        coord_sum += s->coord;
        auto git =
            std::find_if(groups.begin(), groups.end(),
                         [&](const TrackGroup& g) { return g.net == s->net; });
        if (git == groups.end()) {
          groups.push_back({s->net, {s}, 0, 0.0});
        } else {
          git->segs.push_back(s);
        }
      }

      if (groups.size() < 2) {
        c_start = c_end;
        continue;
      }

      const double coord0 = coord_sum / static_cast<double>(c_end - c_start);
      for (TrackGroup& g : groups) {
        ComputeOrderKey(&g);
      }
      std::sort(groups.begin(), groups.end(),
                [](const TrackGroup& a, const TrackGroup& b) {
                  return std::tie(a.shape_class, a.order_key, a.net) <
                         std::tie(b.shape_class, b.order_key, b.net);
                });

      const double search_span = std::max(
          static_cast<double>(groups.size()) * cfg.nudge_distance * 1.5, 120.0);
      double free_min = coord0 - search_span;
      double free_max = coord0 + search_span;

      const Rect query_box = (axis == Axis::kVertical)
                                 ? Rect{coord0 - search_span - cfg.buffer,
                                        cluster_min_k - cfg.buffer,
                                        coord0 + search_span + cfg.buffer,
                                        cluster_max_k + cfg.buffer}
                                 : Rect{cluster_min_k - cfg.buffer,
                                        coord0 - search_span - cfg.buffer,
                                        cluster_max_k + cfg.buffer,
                                        coord0 + search_span + cfg.buffer};

      for (Id bid : index.Query(query_box)) {
        const Rect rb = blocks.at(bid).rect.Pad(cfg.buffer);
        const double b_min_k = (axis == Axis::kVertical) ? rb.t : rb.l;
        const double b_max_k = (axis == Axis::kVertical) ? rb.b : rb.r;
        if (b_max_k <= cluster_min_k + 1e-4 ||
            b_min_k >= cluster_max_k - 1e-4) {
          continue;
        }
        const double b_min_p = (axis == Axis::kVertical) ? rb.l : rb.t;
        const double b_max_p = (axis == Axis::kVertical) ? rb.r : rb.b;
        if (b_max_p <= coord0 + 1e-3) {
          free_min = std::max(free_min, b_max_p + 0.05);
        } else if (b_min_p >= coord0 - 1e-3) {
          free_max = std::min(free_max, b_min_p - 0.05);
        }
      }

      for (std::size_t k = c_start; k < c_end; ++k) {
        const MiddleSegment& s = all_segs[k];
        if (s.prev_prim < s.coord - 1e-3) {
          free_min = std::max(free_min, s.prev_prim + 1.0);
        } else if (s.prev_prim > s.coord + 1e-3) {
          free_max = std::min(free_max, s.prev_prim - 1.0);
        }
        if (s.next_prim < s.coord - 1e-3) {
          free_min = std::max(free_min, s.next_prim + 1.0);
        } else if (s.next_prim > s.coord + 1e-3) {
          free_max = std::min(free_max, s.next_prim - 1.0);
        }
      }

      if (free_max <= free_min + 0.5) {
        c_start = c_end;
        continue;
      }

      const std::size_t m = groups.size();
      const double ideal_span = static_cast<double>(m - 1) * cfg.nudge_distance;
      std::vector<double> targets(m);
      if (free_max - free_min >= ideal_span) {
        const double start = std::clamp(coord0 - 0.5 * ideal_span, free_min,
                                        free_max - ideal_span);
        for (std::size_t i = 0; i < m; ++i) {
          targets[i] = start + static_cast<double>(i) * cfg.nudge_distance;
        }
      } else {
        const double step = (free_max - free_min) / static_cast<double>(m + 1);
        for (std::size_t i = 0; i < m; ++i) {
          targets[i] = free_min + static_cast<double>(i + 1) * step;
        }
      }

      struct Edit {
        Id conn_id = 0;
        std::size_t seg_idx = 0;
        double target = 0.0;
      };
      std::vector<Edit> edits;
      edits.reserve(c_end - c_start);
      for (std::size_t i = 0; i < m; ++i) {
        for (const MiddleSegment* s : groups[i].segs) {
          edits.push_back({s->conn_id, s->seg_idx, targets[i]});
        }
      }
      std::sort(edits.begin(), edits.end(), [](const Edit& a, const Edit& b) {
        return std::tie(a.conn_id, a.seg_idx) < std::tie(b.conn_id, b.seg_idx);
      });

      for (std::size_t e_start = 0; e_start < edits.size();) {
        const Id cid = edits[e_start].conn_id;
        std::size_t e_end = e_start + 1;
        while (e_end < edits.size() && edits[e_end].conn_id == cid) {
          ++e_end;
        }

        Route& route = routes->at(cid);
        std::vector<Point> candidate = route.points;
        for (std::size_t ei = e_start; ei < e_end; ++ei) {
          const std::size_t idx = edits[ei].seg_idx;
          if (idx + 1 < candidate.size()) {
            SetPrimaryCoord(&candidate[idx], axis, edits[ei].target);
            SetPrimaryCoord(&candidate[idx + 1], axis, edits[ei].target);
          }
        }
        if (validate_fn(conns.at(cid), &candidate) &&
            !(candidate == route.points)) {
          route.points = std::move(candidate);
          on_route_updated(cid);
          changed_out->insert(cid);
        }
        e_start = e_end;
      }

      c_start = c_end;
    }

    b_start = b_end;
  }
}

}  // namespace

std::vector<Id> ChannelNudger::Nudge(LocalOrthogonalRouter* router,
                                     const std::vector<Id>& seed_conns,
                                     bool full_graph) {
  if (!router->cfg_.enable_channel_nudging) {
    return {};
  }
  const std::vector<Id> participants = CollectParticipants(
      router->cfg_, router->index_, router->conns_, router->adjacent_,
      router->routes_, seed_conns, full_graph);
  if (participants.size() < 2) {
    return {};
  }

  auto validate_fn = [&](const Conn& conn, std::vector<Point>* pts) {
    return router->ValidateAndSimplifyRoute(conn, pts);
  };
  auto on_updated = [&](Id cid) {
    if (!router->cfg_.reroute_third_party_blocked) {
      return;
    }
    router->ClearSegments(cid);
    const Route& r = router->routes_.at(cid);
    if (r.status == Status::kValid) {
      for (std::size_t j = 1; j < r.points.size(); ++j) {
        router->segments_.Insert((static_cast<std::uint64_t>(cid) << 32) | j,
                                 Box(r.points[j - 1], r.points[j]), true);
      }
    }
  };

  std::set<Id> changed;
  NudgeAxis(router->cfg_, router->index_, router->blocks_, router->conns_,
            participants, Axis::kVertical, &router->routes_, validate_fn,
            on_updated, &changed);
  NudgeAxis(router->cfg_, router->index_, router->blocks_, router->conns_,
            participants, Axis::kHorizontal, &router->routes_, validate_fn,
            on_updated, &changed);
  return {changed.begin(), changed.end()};
}

}  // namespace localroute
