#pragma once
#include "ILocalRouteSolver.h"
#include <stdexcept>
#include <unordered_map>
namespace localroute {
template <class Item> class SpatialGrid {
    using Key = std::uint64_t;
    std::unordered_map<Key, std::vector<Item>> cells;
    std::unordered_map<Item, Rect> boxes;
    std::unordered_map<Item, Rect> overflow;
    double cell;
    int coord(double x) const {
        double c = std::floor(x / cell);
        if (!std::isfinite(c) || std::abs(c) > 1000000000)
            throw std::out_of_range("grid coordinate");
        return int(c);
    }
    struct Range {
        int l, t, r, b;
        std::int64_t count() const { return (1LL + r - l) * (1LL + b - t); }
    };
    Range range(Rect a) const {
        if (!(a.l <= a.r && a.t <= a.b))
            throw std::invalid_argument("box");
        return {coord(a.l), coord(a.t), coord(a.r), coord(a.b)};
    }
    template <class F> static void visit(Range a, F f) {
        for (int y = a.t; y <= a.b; ++y)
            for (int x = a.l; x <= a.r; ++x)
                f((Key(std::uint32_t(x)) << 32) | std::uint32_t(y));
    }

  public:
    explicit SpatialGrid(double c = 256) : cell(c) {
        if (!(c > 0 && std::isfinite(c)))
            throw std::invalid_argument("cell");
    }
    void check(Rect a) const {
        if (range(a).count() > 65536)
            throw std::length_error("oversized block");
    }
    void insert(Item id, Rect a, bool allowOverflow = false) {
        if (allowOverflow && range(a).count() > 1024) {
            overflow.emplace(id, a);
            return;
        }
        check(a);
        if (!boxes.emplace(id, a).second)
            throw std::invalid_argument("duplicate block");
        visit(range(a), [&](Key k) { cells[k].push_back(id); });
    }
    void remove(Item id) {
        if (overflow.erase(id))
            return;
        auto a = range(boxes.at(id));
        visit(a, [&](Key k) {
            auto &v = cells.at(k);
            v.erase(std::find(v.begin(), v.end(), id));
            if (v.empty())
                cells.erase(k);
        });
        boxes.erase(id);
    }
    void update(Item id, Rect a) {
        check(a);
        auto old = range(boxes.at(id)), n = range(a);
        if (old.l == n.l && old.r == n.r && old.t == n.t && old.b == n.b) {
            boxes.at(id) = a;
            return;
        }
        remove(id);
        insert(id, a);
    }
    std::vector<Item> query(Rect a) const {
        std::vector<Item> ids;
        auto g = range(a);
        if (g.count() > std::max<std::int64_t>(32, boxes.size())) {
            for (auto &e : boxes)
                if (e.second.intersects(a))
                    ids.push_back(e.first);
        } else {
            for (int y = g.t; y <= g.b; ++y) {
                bool yEdge = (y == g.t || y == g.b);
                for (int x = g.l; x <= g.r; ++x) {
                    auto it = cells.find((Key(std::uint32_t(x)) << 32) | std::uint32_t(y));
                    if (it == cells.end())
                        continue;
                    if (yEdge || x == g.l || x == g.r) {
                        for (Item id : it->second)
                            if (boxes.at(id).intersects(a))
                                ids.push_back(id);
                    } else {
                        ids.insert(ids.end(), it->second.begin(), it->second.end());
                    }
                }
            }
        }
        // ponytail: scan exceptionally long segments; add a secondary index only if measured hot.
        for (auto &e : overflow)
            if (e.second.intersects(a))
                ids.push_back(e.first);
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
        return ids;
    }
};
using SpatialGridIndex = SpatialGrid<Id>;
} // namespace localroute
