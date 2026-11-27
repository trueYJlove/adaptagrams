#include "channel_nudger.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
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
  bool is_mutable = true;
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
  double min_k = 0.0;
  double max_k = 0.0;
  double coord0 = 0.0;
  int shape_class = 0;
  double order_key = 0.0;
};

struct TrackAnchor {
  NetKey net;
  double coord = 0.0;
  double min_k = 0.0;
  double max_k = 0.0;
  double prev_prim = 0.0;
  double next_prim = 0.0;
  double k_at_seg = 0.0;
  double k_at_next = 0.0;
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

bool IntervalsOverlap(double min_a, double max_a, double min_b, double max_b) {
  return min_a < max_b - 1e-3 && min_b < max_a - 1e-3;
}

bool CrossesOrthoPrimVert(double h_prim0, double h_prim1, double h_k,
                          double v_prim, double v_k0, double v_k1) {
  const double hp_min = std::min(h_prim0, h_prim1);
  const double hp_max = std::max(h_prim0, h_prim1);
  const double vk_min = std::min(v_k0, v_k1);
  const double vk_max = std::max(v_k0, v_k1);
  return v_prim > hp_min + 1e-6 && v_prim < hp_max - 1e-6 &&
         h_k > vk_min + 1e-6 && h_k < vk_max - 1e-6;
}

int CountGroupCrossings(const TrackGroup& group, double cand_coord,
                        const std::vector<const TrackAnchor*>& anchors) {
  int crossings = 0;
  for (const MiddleSegment* s : group.segs) {
    for (const TrackAnchor* a : anchors) {
      if (a->net == group.net) {
        continue;
      }
      if (CrossesOrthoPrimVert(s->prev_prim, cand_coord, s->k_at_seg, a->coord,
                               a->k_at_seg, a->k_at_next)) {
        ++crossings;
      }
      if (CrossesOrthoPrimVert(cand_coord, s->next_prim, s->k_at_next, a->coord,
                               a->k_at_seg, a->k_at_next)) {
        ++crossings;
      }
      if (CrossesOrthoPrimVert(a->prev_prim, a->coord, a->k_at_seg, cand_coord,
                               s->k_at_seg, s->k_at_next)) {
        ++crossings;
      }
      if (CrossesOrthoPrimVert(a->coord, a->next_prim, a->k_at_next, cand_coord,
                               s->k_at_seg, s->k_at_next)) {
        ++crossings;
      }
    }
  }
  return crossings;
}

std::vector<Id> CollectParticipants(
    const Config& cfg, const SpatialGridIndex& index,
    const std::unordered_map<Id, Conn>& conns,
    const std::unordered_map<Id, std::vector<Id>>& adjacent,
    const std::unordered_map<Id, Route>& routes,
    const std::vector<Id>& mutable_conns, bool full_graph,
    std::unordered_set<Id>* mutable_set_out) {
  std::set<Id> selected;
  mutable_set_out->clear();
  if (full_graph) {
    for (const auto& kv : conns) {
      const auto rit = routes.find(kv.first);
      if (rit != routes.end() && rit->second.status == Status::kValid &&
          rit->second.points.size() >= 4) {
        selected.insert(kv.first);
        if (!rit->second.user_modified) {
          mutable_set_out->insert(kv.first);
        }
      }
    }
    return {selected.begin(), selected.end()};
  }

  for (Id cid : mutable_conns) {
    const auto rit = routes.find(cid);
    if (rit == routes.end() || rit->second.status != Status::kValid ||
        rit->second.points.size() < 4) {
      continue;
    }
    selected.insert(cid);
    if (!rit->second.user_modified) {
      mutable_set_out->insert(cid);
    }
  }
  if (mutable_set_out->empty()) {
    return {};
  }

  const std::size_t cap = std::max(kMaxLocalConns, selected.size());
  const double pad = std::max(cfg.nudge_distance * 2.0, 80.0);

  auto collect_from_blocks = [&](const std::vector<Id>& block_ids) {
    for (Id bid : block_ids) {
      if (selected.size() >= cap) {
        return;
      }
      const auto ait = adjacent.find(bid);
      if (ait == adjacent.end()) {
        continue;
      }
      for (Id adj_cid : ait->second) {
        if (selected.size() >= cap) {
          return;
        }
        const auto rit = routes.find(adj_cid);
        if (rit != routes.end() && rit->second.status == Status::kValid &&
            rit->second.points.size() >= 4) {
          selected.insert(adj_cid);
        }
      }
    }
  };

  // Query narrow strips along movable middle segments j in [1, pts.size()-3].
  // Compact local segments (bounding <= 32 grid cells) coalesce into a single
  // SpatialGrid::Query; wider/long-span segments query in bounded narrow
  // strip batches and stop once selected reaches cap.
  std::vector<Id> mut_ids(mutable_set_out->begin(), mutable_set_out->end());
  std::sort(mut_ids.begin(), mut_ids.end());

  std::vector<Rect> short_strips;
  std::vector<Rect> long_strips;
  short_strips.reserve(mut_ids.size() * 2);
  bool has_short_bounds = false;
  Rect short_bounds{0, 0, 0, 0};
  for (Id cid : mut_ids) {
    const auto& pts = routes.at(cid).points;
    for (std::size_t j = 1; j + 2 < pts.size(); ++j) {
      const Rect seg_box = Box(pts[j], pts[j + 1]);
      if (std::max(seg_box.r - seg_box.l, seg_box.b - seg_box.t) <= 600.0) {
        const Rect padded = seg_box.Pad(pad);
        short_strips.push_back(padded);
        if (!has_short_bounds) {
          short_bounds = padded;
          has_short_bounds = true;
        } else {
          short_bounds.l = std::min(short_bounds.l, padded.l);
          short_bounds.t = std::min(short_bounds.t, padded.t);
          short_bounds.r = std::max(short_bounds.r, padded.r);
          short_bounds.b = std::max(short_bounds.b, padded.b);
        }
      } else {
        long_strips.push_back(seg_box.Pad(pad));
      }
    }
  }
  if (has_short_bounds) {
    const std::int64_t cells_w =
        1LL + static_cast<std::int64_t>(std::floor(short_bounds.r / cfg.cell)) -
        static_cast<std::int64_t>(std::floor(short_bounds.l / cfg.cell));
    const std::int64_t cells_h =
        1LL + static_cast<std::int64_t>(std::floor(short_bounds.b / cfg.cell)) -
        static_cast<std::int64_t>(std::floor(short_bounds.t / cfg.cell));
    if (cells_w * cells_h <= 32) {
      collect_from_blocks(index.Query(short_bounds));
    } else {
      constexpr std::size_t kShortBatchSize = 8;
      std::vector<Rect> batch;
      batch.reserve(kShortBatchSize);
      for (std::size_t i = 0; i < short_strips.size() && selected.size() < cap;
           i += kShortBatchSize) {
        batch.clear();
        const std::size_t end =
            std::min(short_strips.size(), i + kShortBatchSize);
        for (std::size_t k = i; k < end; ++k) {
          batch.push_back(short_strips[k]);
        }
        collect_from_blocks(index.QueryUnion(batch));
      }
    }
  }
  if (selected.size() < cap && !long_strips.empty()) {
    constexpr std::size_t kBatchSize = 4;
    const std::size_t max_batches = (selected.size() >= 128) ? 1 : 4;
    std::vector<Rect> batch;
    batch.reserve(kBatchSize);
    std::size_t batches = 0;
    for (std::size_t i = 0; i < long_strips.size() && selected.size() < cap &&
                            batches < max_batches;
         i += kBatchSize, ++batches) {
      batch.clear();
      const std::size_t end = std::min(long_strips.size(), i + kBatchSize);
      for (std::size_t k = i; k < end; ++k) {
        batch.push_back(long_strips[k]);
      }
      collect_from_blocks(index.QueryUnion(batch));
    }
  }
  if (selected.size() < 16) {
    std::vector<Rect> terminal_boxes;
    for (Id cid : mut_ids) {
      const Conn& c = conns.at(cid);
      collect_from_blocks({c.src, c.dst});
      const auto& pts = routes.at(cid).points;
      terminal_boxes.push_back(Box(pts.front(), pts.front()).Pad(pad));
      terminal_boxes.push_back(Box(pts.back(), pts.back()).Pad(pad));
    }
    if (selected.size() < cap && !terminal_boxes.empty()) {
      collect_from_blocks(index.QueryUnion(terminal_boxes));
    }
  }
  return {selected.begin(), selected.end()};
}

std::vector<MiddleSegment> ExtractMiddleSegments(
    const std::vector<Id>& participants,
    const std::unordered_set<Id>& mutable_set,
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
    const bool is_mut = mutable_set.count(cid) != 0;
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
        seg.is_mutable = is_mut;
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

void ComputeGroupStats(TrackGroup* group) {
  const MiddleSegment* rep = group->segs.front();
  double best_span = rep->max_k - rep->min_k;
  double min_k = rep->min_k;
  double max_k = rep->max_k;
  double coord_sum = 0.0;
  for (const MiddleSegment* s : group->segs) {
    min_k = std::min(min_k, s->min_k);
    max_k = std::max(max_k, s->max_k);
    coord_sum += s->coord;
    const double span = s->max_k - s->min_k;
    if (span > best_span + 1e-6) {
      best_span = span;
      rep = s;
    }
  }
  group->min_k = min_k;
  group->max_k = max_k;
  group->coord0 = coord_sum / static_cast<double>(group->segs.size());
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

void ComputeGroupFreeBounds(const Config& cfg, const SpatialGridIndex& index,
                            const std::unordered_map<Id, Block>& blocks,
                            Axis axis, const TrackGroup& group,
                            double search_span, double* free_min_out,
                            double* free_max_out,
                            bool* bounded_both_out = nullptr) {
  double free_min = group.coord0 - search_span;
  double free_max = group.coord0 + search_span;
  bool has_min_bound = false;
  bool has_max_bound = false;
  const Rect query_box = (axis == Axis::kVertical)
                             ? Rect{group.coord0 - search_span - cfg.buffer,
                                    group.min_k - cfg.buffer,
                                    group.coord0 + search_span + cfg.buffer,
                                    group.max_k + cfg.buffer}
                             : Rect{group.min_k - cfg.buffer,
                                    group.coord0 - search_span - cfg.buffer,
                                    group.max_k + cfg.buffer,
                                    group.coord0 + search_span + cfg.buffer};

  for (Id bid : index.Query(query_box)) {
    const Rect rb = blocks.at(bid).rect.Pad(cfg.buffer);
    const double b_min_k = (axis == Axis::kVertical) ? rb.t : rb.l;
    const double b_max_k = (axis == Axis::kVertical) ? rb.b : rb.r;
    if (b_max_k <= group.min_k + 1e-4 || b_min_k >= group.max_k - 1e-4) {
      continue;
    }
    const double b_min_p = (axis == Axis::kVertical) ? rb.l : rb.t;
    const double b_max_p = (axis == Axis::kVertical) ? rb.r : rb.b;
    if (b_max_p <= group.coord0 + 1e-3) {
      free_min = std::max(free_min, b_max_p + 0.05);
      has_min_bound = true;
    } else if (b_min_p >= group.coord0 - 1e-3) {
      free_max = std::min(free_max, b_min_p - 0.05);
      has_max_bound = true;
    }
  }

  for (const MiddleSegment* s : group.segs) {
    if (s->prev_prim < s->coord - 1e-3) {
      free_min = std::max(free_min, s->prev_prim + 1.0);
      has_min_bound = true;
    } else if (s->prev_prim > s->coord + 1e-3) {
      free_max = std::min(free_max, s->prev_prim - 1.0);
      has_max_bound = true;
    }
    if (s->next_prim < s->coord - 1e-3) {
      free_min = std::max(free_min, s->next_prim + 1.0);
      has_min_bound = true;
    } else if (s->next_prim > s->coord + 1e-3) {
      free_max = std::min(free_max, s->next_prim - 1.0);
      has_max_bound = true;
    }
  }

  *free_min_out = free_min;
  *free_max_out = free_max;
  if (bounded_both_out) {
    *bounded_both_out = has_min_bound && has_max_bound;
  }
}

double PickNearestFreeTrack(const Config& cfg, const TrackGroup& group,
                            double free_min, double free_max,
                            const std::vector<TrackAnchor>& anchors,
                            bool allow_edge_fallback = true) {
  const double d = cfg.nudge_distance;
  std::vector<const TrackAnchor*> relevant;
  for (const TrackAnchor& a : anchors) {
    if (a.net == group.net) {
      continue;
    }
    if (IntervalsOverlap(a.min_k, a.max_k, group.min_k, group.max_k) &&
        a.coord >= free_min - d && a.coord <= free_max + d) {
      relevant.push_back(&a);
    }
  }
  if (relevant.empty()) {
    return std::clamp(group.coord0, free_min, free_max);
  }

  std::vector<double> cands;
  cands.reserve(relevant.size() * 3 + 3);
  cands.push_back(std::clamp(group.coord0, free_min, free_max));
  if (allow_edge_fallback) {
    cands.push_back(free_min);
    cands.push_back(free_max);
  }

  std::vector<double> sorted_coords;
  sorted_coords.reserve(relevant.size());
  for (const TrackAnchor* a : relevant) {
    sorted_coords.push_back(a->coord);
    const double left_cand = a->coord - d;
    const double right_cand = a->coord + d;
    if (left_cand >= free_min - 1e-6 && left_cand <= free_max + 1e-6) {
      cands.push_back(std::clamp(left_cand, free_min, free_max));
    }
    if (right_cand >= free_min - 1e-6 && right_cand <= free_max + 1e-6) {
      cands.push_back(std::clamp(right_cand, free_min, free_max));
    }
  }
  std::sort(sorted_coords.begin(), sorted_coords.end());
  for (std::size_t i = 1; i < sorted_coords.size(); ++i) {
    const double mid = 0.5 * (sorted_coords[i - 1] + sorted_coords[i]);
    if (mid >= free_min && mid <= free_max) {
      cands.push_back(mid);
    }
  }

  // Sort candidates by (displacement, cand) ascending and remove exact
  // duplicates so the first clear zero-crossing candidate is provably optimal.
  std::sort(cands.begin(), cands.end());
  cands.erase(std::unique(cands.begin(), cands.end()), cands.end());
  std::sort(cands.begin(), cands.end(), [&](double a, double b) {
    const double da = std::abs(a - group.coord0);
    const double db = std::abs(b - group.coord0);
    if (da != db) {
      return da < db;
    }
    return a < b;
  });

  auto min_dist_to_anchors = [&](double x) {
    double min_dist = 1e9;
    for (const TrackAnchor* a : relevant) {
      min_dist = std::min(min_dist, std::abs(x - a->coord));
    }
    return min_dist;
  };

  bool found_clear = false;
  std::tuple<int, double, double> best_clear{0, 1e18, 0.0};
  double best_clear_x = group.coord0;

  double best_tight_dist = -1.0;
  std::tuple<int, double, double> best_tight{0, 1e18, 0.0};
  double best_tight_x = group.coord0;

  double g_p_min = free_min;
  double g_p_max = free_max;
  for (const MiddleSegment* s : group.segs) {
    g_p_min = std::min({g_p_min, s->prev_prim, s->next_prim});
    g_p_max = std::max({g_p_max, s->prev_prim, s->next_prim});
  }
  std::vector<const TrackAnchor*> cross_anchors;
  cross_anchors.reserve(32);
  for (const TrackAnchor& a : anchors) {
    if (a.net == group.net) {
      continue;
    }
    const double a_p_min = std::min({a.prev_prim, a.coord, a.next_prim});
    const double a_p_max = std::max({a.prev_prim, a.coord, a.next_prim});
    if (a_p_max > g_p_min + 1e-6 && g_p_max > a_p_min + 1e-6 &&
        a.max_k > group.min_k - 1e-3 && group.max_k > a.min_k - 1e-3) {
      cross_anchors.push_back(&a);
    }
  }

  for (double cand : cands) {
    if (!allow_edge_fallback &&
        std::abs(cand - group.coord0) >
            cfg.max_nudge_distance + cfg.nudge_distance) {
      continue;
    }
    const double md = min_dist_to_anchors(cand);
    if (md >= d - 1e-4) {
      const int cross = cross_anchors.empty()
                            ? 0
                            : CountGroupCrossings(group, cand, cross_anchors);
      if (cross == 0) {
        return cand;
      }
      const double disp = std::abs(cand - group.coord0);
      const std::tuple<int, double, double> score{cross, disp, cand};
      if (!found_clear || score < best_clear) {
        found_clear = true;
        best_clear = score;
        best_clear_x = cand;
      }
    } else if (!found_clear) {
      const double disp = std::abs(cand - group.coord0);
      if (md > best_tight_dist + 1e-4) {
        best_tight_dist = md;
        const int cross = cross_anchors.empty()
                              ? 0
                              : CountGroupCrossings(group, cand, cross_anchors);
        best_tight = {cross, disp, cand};
        best_tight_x = cand;
      } else if (std::abs(md - best_tight_dist) <= 1e-4) {
        const int cross = cross_anchors.empty()
                              ? 0
                              : CountGroupCrossings(group, cand, cross_anchors);
        const std::tuple<int, double, double> score{cross, disp, cand};
        if (score < best_tight) {
          best_tight = score;
          best_tight_x = cand;
        }
      }
    }
  }

  if (found_clear) {
    return best_clear_x;
  }
  if (best_tight_dist > 0.5) {
    return best_tight_x;
  }
  return group.coord0;
}

void NudgeAxis(
    const Config& cfg, const SpatialGridIndex& index,
    const std::unordered_map<Id, Block>& blocks,
    const std::unordered_map<Id, Conn>& conns,
    const std::vector<Id>& participants,
    const std::unordered_set<Id>& mutable_set, bool full_graph, Axis axis,
    std::unordered_map<Id, Route>* routes,
    const std::function<bool(const Conn&, std::vector<Point>*)>& validate_fn,
    const std::function<bool(const Conn&, const std::vector<Point>&,
                             std::size_t)>& validate_seg_fn,
    const std::function<void(Id)>& on_route_updated,
    std::set<Id>* changed_out) {
  std::vector<MiddleSegment> all_segs =
      ExtractMiddleSegments(participants, mutable_set, conns, *routes, axis);
  if (all_segs.size() < 2) {
    return;
  }

  std::vector<TrackAnchor> base_fixed_anchors;
  for (const MiddleSegment& s : all_segs) {
    if (!s.is_mutable) {
      base_fixed_anchors.push_back({s.net, s.coord, s.min_k, s.max_k,
                                    s.prev_prim, s.next_prim, s.k_at_seg,
                                    s.k_at_next});
    }
  }

  std::sort(all_segs.begin(), all_segs.end(),
            [](const MiddleSegment& a, const MiddleSegment& b) {
              if (a.coord != b.coord) {
                return a.coord < b.coord;
              }
              return std::tie(a.min_k, a.max_k, a.net, a.conn_id, a.seg_idx) <
                     std::tie(b.min_k, b.max_k, b.net, b.conn_id, b.seg_idx);
            });

  // On cold-start full pass, routes start un-nudged so coincident tracks sit
  // within <= 1.5 px; on incremental local passes, include tracks within
  // max(nudge_distance, 0.5 * max_nudge_distance + 0.5) so newly placed routes
  // detect nearby fixed tracks in elastically widened corridors without merging
  // unrelated columns.
  const double band_gap =
      full_graph ? 1.5
                 : std::max(1.5, std::max(cfg.nudge_distance,
                                          0.5 * cfg.max_nudge_distance + 0.5) -
                                     1e-3);
  const double max_band_span =
      full_graph ? 2.5 : std::max(16.0, cfg.nudge_distance * 4.0);
  for (std::size_t b_start = 0; b_start < all_segs.size();) {
    const double anchor = all_segs[b_start].coord;
    std::size_t b_end = b_start + 1;
    while (b_end < all_segs.size() &&
           all_segs[b_end].coord - all_segs[b_end - 1].coord < band_gap &&
           all_segs[b_end].coord - anchor <= max_band_span) {
      ++b_end;
    }
    if (b_end - b_start < 2) {
      b_start = b_end;
      continue;
    }
    if (!full_graph) {
      bool band_has_mut = false;
      for (std::size_t k = b_start; k < b_end; ++k) {
        if (all_segs[k].is_mutable) {
          band_has_mut = true;
          break;
        }
      }
      if (!band_has_mut) {
        b_start = b_end;
        continue;
      }
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
      if (!full_graph) {
        bool cluster_has_mut = false;
        for (std::size_t k = c_start; k < c_end; ++k) {
          if (all_segs[k].is_mutable) {
            cluster_has_mut = true;
            break;
          }
        }
        if (!cluster_has_mut) {
          c_start = c_end;
          continue;
        }
      }

      std::vector<TrackGroup> all_groups;
      for (std::size_t k = c_start; k < c_end; ++k) {
        const MiddleSegment* s = &all_segs[k];
        auto git =
            std::find_if(all_groups.begin(), all_groups.end(),
                         [&](const TrackGroup& g) { return g.net == s->net; });
        if (git == all_groups.end()) {
          all_groups.push_back({s->net, {s}, 0.0, 0.0, 0.0, 0, 0.0});
        } else {
          git->segs.push_back(s);
        }
      }

      if (all_groups.size() < 2) {
        c_start = c_end;
        continue;
      }

      std::vector<TrackGroup> mutable_groups;
      std::vector<std::pair<const MiddleSegment*, double>> edits_src;
      for (TrackGroup& g : all_groups) {
        ComputeGroupStats(&g);
        const MiddleSegment* fixed_rep = nullptr;
        for (const MiddleSegment* s : g.segs) {
          if (!s->is_mutable) {
            fixed_rep = s;
            break;
          }
        }
        if (fixed_rep != nullptr) {
          // A same-source fixed sibling anchors the trunk; never shift it,
          // only align mutable siblings onto that shared trunk if needed.
          for (const MiddleSegment* s : g.segs) {
            if (s->is_mutable && std::abs(s->coord - fixed_rep->coord) > 1e-6) {
              edits_src.emplace_back(s, fixed_rep->coord);
            }
          }
        } else {
          mutable_groups.push_back(std::move(g));
        }
      }

      if (!mutable_groups.empty()) {
        std::sort(mutable_groups.begin(), mutable_groups.end(),
                  [](const TrackGroup& a, const TrackGroup& b) {
                    return std::tie(a.shape_class, a.order_key, a.net) <
                           std::tie(b.shape_class, b.order_key, b.net);
                  });

        const double search_span = std::max(
            static_cast<double>(all_groups.size()) * cfg.nudge_distance * 1.5,
            120.0);

        bool can_use_balanced = (mutable_groups.size() >= 2);
        if (can_use_balanced && !full_graph) {
          bool has_overlap = false;
          for (std::size_t i = 0; i < mutable_groups.size() && !has_overlap;
               ++i) {
            for (std::size_t j = i + 1; j < mutable_groups.size(); ++j) {
              if (IntervalsOverlap(
                      mutable_groups[i].min_k, mutable_groups[i].max_k,
                      mutable_groups[j].min_k, mutable_groups[j].max_k) &&
                  std::abs(mutable_groups[i].coord0 -
                           mutable_groups[j].coord0) <
                      cfg.nudge_distance - 1e-3) {
                has_overlap = true;
                break;
              }
            }
          }
          can_use_balanced = has_overlap;
        }

        if (can_use_balanced) {
          TrackGroup union_mut;
          double coord_sum = 0.0;
          std::size_t seg_count = 0;
          union_mut.min_k = mutable_groups.front().min_k;
          union_mut.max_k = mutable_groups.front().max_k;
          for (const TrackGroup& g : mutable_groups) {
            union_mut.min_k = std::min(union_mut.min_k, g.min_k);
            union_mut.max_k = std::max(union_mut.max_k, g.max_k);
            for (const MiddleSegment* s : g.segs) {
              union_mut.segs.push_back(s);
              coord_sum += s->coord;
              ++seg_count;
            }
          }
          union_mut.coord0 = coord_sum / static_cast<double>(seg_count);
          double free_min = 0.0;
          double free_max = 0.0;
          bool bounded_both = false;
          ComputeGroupFreeBounds(cfg, index, blocks, axis, union_mut,
                                 search_span, &free_min, &free_max,
                                 &bounded_both);

          if (free_max > free_min + 0.5) {
            const std::size_t m = mutable_groups.size();
            const double avail = free_max - free_min;
            const double min_span =
                static_cast<double>(m - 1) * cfg.nudge_distance;
            std::vector<double> targets(m);
            if (avail >= min_span) {
              const double step =
                  bounded_both
                      ? std::clamp((avail * 0.5) / static_cast<double>(m),
                                   cfg.nudge_distance, cfg.max_nudge_distance)
                      : cfg.nudge_distance;
              const double spread = static_cast<double>(m - 1) * step;
              const double start = std::clamp(union_mut.coord0 - 0.5 * spread,
                                              free_min, free_max - spread);
              for (std::size_t i = 0; i < m; ++i) {
                targets[i] = start + static_cast<double>(i) * step;
              }
            } else {
              const double step = avail / static_cast<double>(m + 1);
              for (std::size_t i = 0; i < m; ++i) {
                targets[i] = free_min + static_cast<double>(i + 1) * step;
              }
            }

            bool hits_fixed = false;
            for (std::size_t i = 0; i < m && !hits_fixed; ++i) {
              const TrackGroup& g = mutable_groups[i];
              for (const TrackAnchor& a : base_fixed_anchors) {
                if (a.net == g.net) {
                  continue;
                }
                if (IntervalsOverlap(a.min_k, a.max_k, g.min_k, g.max_k) &&
                    std::abs(targets[i] - a.coord) <
                        cfg.nudge_distance - 1e-4) {
                  hits_fixed = true;
                  break;
                }
              }
            }
            if (!hits_fixed) {
              for (std::size_t i = 0; i < m; ++i) {
                for (const MiddleSegment* s : mutable_groups[i].segs) {
                  edits_src.emplace_back(s, targets[i]);
                }
              }
              mutable_groups.clear();
            }
          }
        }

        if (!mutable_groups.empty()) {
          // Place each mutable group onto its nearest free non-crossing orbit
          // against read-only fixed anchors and previously placed mutable ones.
          std::vector<TrackAnchor> occupied = base_fixed_anchors;
          for (const TrackGroup& g : mutable_groups) {
            bool dist_conflict = false;
            bool cross_conflict = false;
            for (const TrackAnchor& a : occupied) {
              if (a.net == g.net) {
                continue;
              }
              if (IntervalsOverlap(a.min_k, a.max_k, g.min_k, g.max_k) &&
                  std::abs(g.coord0 - a.coord) < cfg.nudge_distance - 1e-3) {
                dist_conflict = true;
                break;
              }
            }
            if (!dist_conflict &&
                cfg.max_nudge_distance > cfg.nudge_distance + 1e-6) {
              for (const TrackAnchor& a : occupied) {
                if (a.net == g.net ||
                    std::abs(g.coord0 - a.coord) > band_gap + 1e-3 ||
                    !IntervalsOverlap(a.min_k, a.max_k, g.min_k, g.max_k)) {
                  continue;
                }
                for (const MiddleSegment* s : g.segs) {
                  if (CrossesOrthoPrimVert(s->prev_prim, g.coord0, s->k_at_seg,
                                           a.coord, a.k_at_seg, a.k_at_next) ||
                      CrossesOrthoPrimVert(g.coord0, s->next_prim, s->k_at_next,
                                           a.coord, a.k_at_seg, a.k_at_next) ||
                      CrossesOrthoPrimVert(a.prev_prim, a.coord, a.k_at_seg,
                                           g.coord0, s->k_at_seg,
                                           s->k_at_next) ||
                      CrossesOrthoPrimVert(a.coord, a.next_prim, a.k_at_next,
                                           g.coord0, s->k_at_seg,
                                           s->k_at_next)) {
                    cross_conflict = true;
                    break;
                  }
                }
                if (cross_conflict) {
                  break;
                }
              }
            }

            double chosen = g.coord0;
            if (dist_conflict || cross_conflict) {
              double free_min = 0.0;
              double free_max = 0.0;
              ComputeGroupFreeBounds(cfg, index, blocks, axis, g, search_span,
                                     &free_min, &free_max);
              if (free_max > free_min + 0.5) {
                chosen =
                    PickNearestFreeTrack(cfg, g, free_min, free_max, occupied,
                                         /*allow_edge_fallback=*/dist_conflict);
              }
            }
            for (const MiddleSegment* s : g.segs) {
              if (std::abs(s->coord - chosen) > 1e-6) {
                edits_src.emplace_back(s, chosen);
              }
              occupied.push_back({g.net, chosen, s->min_k, s->max_k,
                                  s->prev_prim, s->next_prim, s->k_at_seg,
                                  s->k_at_next});
            }
          }
        }
      }

      if (!edits_src.empty()) {
        std::sort(edits_src.begin(), edits_src.end(),
                  [](const auto& a, const auto& b) {
                    return std::tie(a.first->conn_id, a.first->seg_idx) <
                           std::tie(b.first->conn_id, b.first->seg_idx);
                  });

        for (std::size_t e_start = 0; e_start < edits_src.size();) {
          const Id cid = edits_src[e_start].first->conn_id;
          std::size_t e_end = e_start + 1;
          while (e_end < edits_src.size() &&
                 edits_src[e_end].first->conn_id == cid) {
            ++e_end;
          }

          if (mutable_set.count(cid)) {
            Route& route = routes->at(cid);
            std::vector<Point> candidate = route.points;
            for (std::size_t ei = e_start; ei < e_end; ++ei) {
              const std::size_t idx = edits_src[ei].first->seg_idx;
              if (idx + 1 < candidate.size()) {
                SetPrimaryCoord(&candidate[idx], axis, edits_src[ei].second);
                SetPrimaryCoord(&candidate[idx + 1], axis,
                                edits_src[ei].second);
              }
            }
            if (!(candidate == route.points)) {
              bool ok = false;
              if (e_end - e_start == 1) {
                const std::size_t seg_idx = edits_src[e_start].first->seg_idx;
                const double new_coord = edits_src[e_start].second;
                if (std::isfinite(new_coord) && std::abs(new_coord) <= 5e7 &&
                    seg_idx >= 1 && seg_idx + 2 < candidate.size() &&
                    !(candidate[seg_idx - 1] == candidate[seg_idx]) &&
                    !(candidate[seg_idx] == candidate[seg_idx + 1]) &&
                    !(candidate[seg_idx + 1] == candidate[seg_idx + 2])) {
                  ok = validate_seg_fn(conns.at(cid), candidate, seg_idx);
                } else {
                  ok = validate_fn(conns.at(cid), &candidate);
                }
              } else {
                ok = validate_fn(conns.at(cid), &candidate);
              }
              if (ok && !(candidate == route.points)) {
                route.points = std::move(candidate);
                on_route_updated(cid);
                changed_out->insert(cid);
              }
            }
          }
          e_start = e_end;
        }
      }

      c_start = c_end;
    }

    b_start = b_end;
  }
}

}  // namespace

std::vector<Id> ChannelNudger::Nudge(LocalOrthogonalRouter* router,
                                     const std::vector<Id>& mutable_conns,
                                     bool full_graph) {
  if (!router->cfg_.enable_channel_nudging) {
    return {};
  }
  std::unordered_set<Id> mutable_set;
  const std::vector<Id> participants = CollectParticipants(
      router->cfg_, router->index_, router->conns_, router->adjacent_,
      router->routes_, mutable_conns, full_graph, &mutable_set);
  if (participants.size() < 2 || mutable_set.empty()) {
    return {};
  }

  auto validate_fn = [&](const Conn& conn, std::vector<Point>* pts) {
    return router->ValidateAndSimplifyRoute(conn, pts);
  };
  auto validate_seg_fn = [&](const Conn& conn, const std::vector<Point>& pts,
                             std::size_t seg_idx) {
    return router->ValidateNudgedMiddleSegment(conn, pts, seg_idx);
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
            participants, mutable_set, full_graph, Axis::kVertical,
            &router->routes_, validate_fn, validate_seg_fn, on_updated,
            &changed);
  NudgeAxis(router->cfg_, router->index_, router->blocks_, router->conns_,
            participants, mutable_set, full_graph, Axis::kHorizontal,
            &router->routes_, validate_fn, validate_seg_fn, on_updated,
            &changed);
  return {changed.begin(), changed.end()};
}

}  // namespace localroute
