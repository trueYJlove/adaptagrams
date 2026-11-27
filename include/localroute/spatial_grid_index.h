#ifndef LOCALROUTE_INCLUDE_LOCALROUTE_SPATIAL_GRID_INDEX_H_
#define LOCALROUTE_INCLUDE_LOCALROUTE_SPATIAL_GRID_INDEX_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "localroute/local_route_solver.h"

namespace localroute {

template <class Item>
class SpatialGrid {
 public:
  explicit SpatialGrid(double cell_size = 256.0) : cell_(cell_size) {
    if (!(cell_size > 0.0 && std::isfinite(cell_size))) {
      throw std::invalid_argument("cell");
    }
  }

  void Check(Rect a) const {
    if (GetRange(a).Count() > 65536) {
      throw std::length_error("oversized block");
    }
  }

  void Insert(Item id, Rect a, bool allow_overflow = false) {
    if (allow_overflow && GetRange(a).Count() > 1024) {
      if (boxes_.count(id) || !overflow_.emplace(id, a).second) {
        throw std::invalid_argument("duplicate block");
      }
      return;
    }
    Check(a);
    if (overflow_.count(id) || !boxes_.emplace(id, a).second) {
      throw std::invalid_argument("duplicate block");
    }
    Visit(GetRange(a), [&](Key k) { cells_[k].push_back(id); });
  }

  void Remove(Item id) {
    if (overflow_.erase(id)) {
      return;
    }
    const Range a = GetRange(boxes_.at(id));
    Visit(a, [&](Key k) {
      auto& v = cells_.at(k);
      v.erase(std::find(v.begin(), v.end(), id));
      if (v.empty()) {
        cells_.erase(k);
      }
    });
    boxes_.erase(id);
  }

  void Update(Item id, Rect a) {
    Check(a);
    const Range old_range = GetRange(boxes_.at(id));
    const Range new_range = GetRange(a);
    if (old_range.l == new_range.l && old_range.r == new_range.r &&
        old_range.t == new_range.t && old_range.b == new_range.b) {
      boxes_.at(id) = a;
      return;
    }
    Remove(id);
    Insert(id, a);
  }

  std::vector<Item> Query(Rect a) const {
    std::vector<Item> ids;
    const Range g = GetRange(a);
    if (g.Count() > std::max<std::int64_t>(32, boxes_.size())) {
      for (const auto& entry : boxes_) {
        if (entry.second.Intersects(a)) {
          ids.push_back(entry.first);
        }
      }
    } else {
      for (int y = g.t; y <= g.b; ++y) {
        const bool y_edge = (y == g.t || y == g.b);
        for (int x = g.l; x <= g.r; ++x) {
          const auto it = cells_.find(PackKey(x, y));
          if (it == cells_.end()) {
            continue;
          }
          if (y_edge || x == g.l || x == g.r) {
            for (Item id : it->second) {
              if (boxes_.at(id).Intersects(a)) {
                ids.push_back(id);
              }
            }
          } else {
            ids.insert(ids.end(), it->second.begin(), it->second.end());
          }
        }
      }
    }
    for (const auto& entry : overflow_) {
      if (entry.second.Intersects(a)) {
        ids.push_back(entry.first);
      }
    }
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    return ids;
  }

  std::vector<Item> QueryUnion(const std::vector<Rect>& areas) const {
    if (areas.empty()) {
      return {};
    }
    if (areas.size() == 1) {
      return Query(areas.front());
    }
    std::vector<std::pair<Key, std::size_t>> cell_areas;
    cell_areas.reserve(areas.size() * 4);
    for (std::size_t i = 0; i < areas.size(); ++i) {
      const Range g = GetRange(areas[i]);
      if (g.Count() > std::max<std::int64_t>(32, boxes_.size())) {
        std::vector<Item> ids;
        for (const auto& entry : boxes_) {
          for (const Rect& box : areas) {
            if (entry.second.Intersects(box)) {
              ids.push_back(entry.first);
              break;
            }
          }
        }
        for (const auto& entry : overflow_) {
          for (const Rect& box : areas) {
            if (entry.second.Intersects(box)) {
              ids.push_back(entry.first);
              break;
            }
          }
        }
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
        return ids;
      }
      Visit(g, [&](Key k) { cell_areas.emplace_back(k, i); });
    }
    std::sort(
        cell_areas.begin(), cell_areas.end(), [](const auto& a, const auto& b) {
          return a.first != b.first ? a.first < b.first : a.second < b.second;
        });

    std::vector<Item> ids;
    for (std::size_t c_start = 0; c_start < cell_areas.size();) {
      const Key k = cell_areas[c_start].first;
      std::size_t c_end = c_start + 1;
      while (c_end < cell_areas.size() && cell_areas[c_end].first == k) {
        ++c_end;
      }
      const auto it = cells_.find(k);
      if (it != cells_.end()) {
        for (Item id : it->second) {
          const Rect& r = boxes_.at(id);
          for (std::size_t ci = c_start; ci < c_end; ++ci) {
            if (r.Intersects(areas[cell_areas[ci].second])) {
              ids.push_back(id);
              break;
            }
          }
        }
      }
      c_start = c_end;
    }
    for (const auto& entry : overflow_) {
      for (const Rect& box : areas) {
        if (entry.second.Intersects(box)) {
          ids.push_back(entry.first);
          break;
        }
      }
    }
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    return ids;
  }

 private:
  using Key = std::uint64_t;

  struct Range {
    int l = 0;
    int t = 0;
    int r = 0;
    int b = 0;

    std::int64_t Count() const { return (1LL + r - l) * (1LL + b - t); }
  };

  static Key PackKey(int x, int y) {
    return (static_cast<Key>(static_cast<std::uint32_t>(x)) << 32) |
           static_cast<std::uint32_t>(y);
  }

  int Coord(double x) const {
    const double c = std::floor(x / cell_);
    if (!std::isfinite(c) || std::abs(c) > 1000000000.0) {
      throw std::out_of_range("grid coordinate");
    }
    return static_cast<int>(c);
  }

  Range GetRange(Rect a) const {
    if (!(a.l <= a.r && a.t <= a.b)) {
      throw std::invalid_argument("box");
    }
    return {Coord(a.l), Coord(a.t), Coord(a.r), Coord(a.b)};
  }

  template <class Fn>
  static void Visit(Range a, Fn fn) {
    for (int y = a.t; y <= a.b; ++y) {
      for (int x = a.l; x <= a.r; ++x) {
        fn(PackKey(x, y));
      }
    }
  }

  std::unordered_map<Key, std::vector<Item>> cells_;
  std::unordered_map<Item, Rect> boxes_;
  std::unordered_map<Item, Rect> overflow_;
  double cell_ = 256.0;
};

using SpatialGridIndex = SpatialGrid<Id>;

}  // namespace localroute

#endif  // LOCALROUTE_INCLUDE_LOCALROUTE_SPATIAL_GRID_INDEX_H_
