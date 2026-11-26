#include "LocalOrthogonalRouter.h"
#include <atomic>
#include <chrono>
#include <fstream>
#include <future>
#include <set>
#include <thread>
#include <tuple>
#include <utility>
namespace localroute {
namespace {
using Clock = std::chrono::steady_clock;
struct Stopwatch {
    double *total;
    Clock::time_point start;
    explicit Stopwatch(double *out) : total(out), start(out ? Clock::now() : Clock::time_point()) {}
    ~Stopwatch() {
        if (total)
            *total += std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    }
};
std::vector<Point> elbow(Point a, Point b, bool horizontal) {
    if (a == b)
        return {a};
    if (a.x == b.x || a.y == b.y)
        return {a, b};
    return {a, horizontal ? Point{b.x, a.y} : Point{a.x, b.y}, b};
}
const Pin &pin(const Block &b, Id id) {
    for (auto &p : b.pins)
        if (p.id == id)
            return p;
    throw std::invalid_argument("unknown pin");
}
Point position(Rect a, Pin p) {
    return {p.x == 1 ? a.r : a.l + p.x * (a.r - a.l), p.y == 1 ? a.b : a.t + p.y * (a.b - a.t)};
}
unsigned dir(Point a, Point b) {
    if (a == b)
        return 0;
    if (a.x == b.x)
        return a.y < b.y ? Down : Up;
    if (a.y == b.y)
        return a.x < b.x ? Right : Left;
    return 0;
}
bool crosses(Point a, Point b, Rect r) {
    auto s = box(a, b);
    return s.r > r.l && s.l < r.r && s.b > r.t && s.t < r.b;
}
void checkBox(Rect a) {
    if (!(a.l < a.r && a.t < a.b &&
          std::max({std::abs(a.l), std::abs(a.r), std::abs(a.t), std::abs(a.b)}) <= 1e7))
        throw std::invalid_argument("invalid block rectangle");
}
bool canExit(Point end, Point next, Rect own, Rect buffered) {
    return !buffered.inside(next) && (own.inside(end) || !crosses(end, next, own));
}
bool finitePoint(Point p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::max(std::abs(p.x), std::abs(p.y)) <= 5e7;
}
} // namespace
LocalOrthogonalRouter::LocalOrthogonalRouter(Config c, std::shared_ptr<const ILocalRouteSolver> s)
    : cfg(c), index(c.cell), solver(s ? s : lazyHananAStarSolver()), segments(c.cell) {
    if (!(c.cell >= 1 && c.padding > 0 && c.padding <= 1e5 && c.buffer >= 0 && c.buffer <= 1e4 &&
          c.penalty > 0 && std::isfinite(c.penalty) && c.nudgeDistance > 0 &&
          std::isfinite(c.nudgeDistance) && c.budget >= 2 && c.retries <= 8))
        throw std::invalid_argument("invalid local router config");
}
void LocalOrthogonalRouter::addBlock(Block b) {
    checkBox(b.rect);
    std::set<Id> seen;
    for (auto p : b.pins)
        if (!p.id || p.id == 2147483647u || !(p.x >= 0 && p.x <= 1 && p.y >= 0 && p.y <= 1) || !p.dirs ||
            (p.dirs & ~15u) || !seen.insert(p.id).second)
            throw std::invalid_argument("invalid pin");
    index.insert(b.id, b.rect);
    Id id = b.id;
    auto rect = b.rect;
    blocks.emplace(id, std::move(b));
    if (cfg.rerouteThirdPartyBlocked)
        invalidateObstacle(id, rect);
}
void LocalOrthogonalRouter::addConnector(Conn c) {
    pin(blocks.at(c.src), c.sp);
    pin(blocks.at(c.dst), c.dp);
    if (!conns.emplace(c.id, c).second)
        throw std::invalid_argument("duplicate connector");
    adjacent[c.src].push_back(c.id);
    if (c.src != c.dst)
        adjacent[c.dst].push_back(c.id);
}
void LocalOrthogonalRouter::removeConnector(Id id) {
    auto c = conns.at(id);
    auto unlink = [&](Id k) {
        auto &v = adjacent.at(k);
        v.erase(std::find(v.begin(), v.end(), id));
    };
    unlink(c.src);
    if (c.src != c.dst)
        unlink(c.dst);
    clearSegments(id);
    routes.erase(id);
    dirty.erase(id);
    conns.erase(id);
}
void LocalOrthogonalRouter::removeBlock(Id id) {
    blocks.at(id);
    auto ids = adjacent[id];
    for (auto c : ids)
        removeConnector(c);
    adjacent.erase(id);
    index.remove(id);
    blocks.erase(id);
}
void LocalOrthogonalRouter::moveBlocksBatch(const std::vector<std::pair<Id, Rect>> &moves) {
    for (auto &m : moves) {
        blocks.at(m.first);
        checkBox(m.second);
        index.check(m.second);
    }
    for (auto &m : moves) {
        index.update(m.first, m.second);
        blocks.at(m.first).rect = m.second;
    }
    for (auto &m : moves) {
        auto it = adjacent.find(m.first);
        if (it != adjacent.end())
            for (Id id : it->second)
                mark(id);
        if (cfg.rerouteThirdPartyBlocked)
            invalidateObstacle(m.first, blocks.at(m.first).rect);
    }
}
void LocalOrthogonalRouter::moveBlock(Id id, double x, double y) {
    auto a = block(id).rect;
    moveBlocksBatch({{id, {a.l + x, a.t + y, a.r + x, a.b + y}}});
}
std::vector<Point> LocalOrthogonalRouter::preview(Id id) const {
    const auto &c = conns.at(id);
    const auto &a = blocks.at(c.src), &b = blocks.at(c.dst);
    return elbow(position(a.rect, pin(a, c.sp)), position(b.rect, pin(b, c.dp)), true);
}
void LocalOrthogonalRouter::mark(Id id) {
    dirty.insert(id);
    routes[id].status = Status::Pending;
}
void LocalOrthogonalRouter::invalidateObstacle(Id blockId, Rect rect) {
    rect = rect.pad(cfg.buffer);
    for (auto key : segments.query(rect)) {
        Id id = Id(key >> 32);
        const auto &c = conns.at(id);
        if (c.src == blockId || c.dst == blockId || dirty.count(id))
            continue;
        const auto &p = routes.at(id).points;
        auto j = std::uint32_t(key);
        if (crosses(p[j - 1], p[j], rect))
            mark(id);
    }
}
void LocalOrthogonalRouter::clearSegments(Id id) {
    if (!cfg.rerouteThirdPartyBlocked)
        return;
    auto it = routes.find(id);
    if (it != routes.end() && it->second.method != RouteMethod::None)
        for (size_t j = 1; j < it->second.points.size(); ++j)
            segments.remove((std::uint64_t(id) << 32) | j);
}
void LocalOrthogonalRouter::publish(Id id, Route route) {
    if (cfg.rerouteThirdPartyBlocked) {
        clearSegments(id);
        if (route.status == Status::Valid)
            for (size_t j = 1; j < route.points.size(); ++j)
                segments.insert((std::uint64_t(id) << 32) | j, box(route.points[j - 1], route.points[j]), true);
    }
    routes[id] = std::move(route);
    dirty.erase(id);
}
Route LocalOrthogonalRouter::solveOne(const Conn &c, bool settled) const {
    const auto &src = blocks.at(c.src);
    const auto &dst = blocks.at(c.dst);
    auto sp = pin(src, c.sp), dp = pin(dst, c.dp);
    Point a = position(src.rect, sp), b = position(dst.rect, dp);
    Route out;
    if (a == b) {
        out.outcome = Outcome::CoincidentPins;
        return out;
    }
    out.points = elbow(a, b, true);
    out.status = Status::DisplayOnly;
    Profile *profile = cfg.profile ? &out.profile : nullptr;
    auto query = [&](Rect rect) {
        Stopwatch clock(profile ? &profile->queryMs : nullptr);
        return index.query(rect);
    };
    auto aa = src.rect.pad(cfg.buffer), bb = dst.rect.pad(cfg.buffer);
    // A terminal buried inside another buffered rectangle has no legal exit.
    for (auto end : {std::make_pair(a, c.src), std::make_pair(b, c.dst)})
        for (Id id : query(box(end.first, end.first).pad(cfg.buffer)))
            if (id != end.second && blocks.at(id).rect.pad(cfg.buffer).inside(end.first)) {
                out.outcome = Outcome::BuriedPin;
                return out;
            }
    std::set<Id> required{c.src, c.dst};
    auto accept = [&](std::vector<Point> p, RouteMethod method) {
        Stopwatch clock(profile ? &profile->validationMs : nullptr);
        auto invalid = [&] {
            out.outcome = Outcome::InvalidCandidate;
            if (profile && method == RouteMethod::Native)
                ++profile->invalidAttempts;
            return false;
        };
        if (!std::all_of(p.begin(), p.end(), finitePoint))
            return invalid();
        p.erase(std::unique(p.begin(), p.end()), p.end());
        // Merge redundant collinear points before checking the terminal exits.
        if (method != RouteMethod::Native) {
            size_t count = 0;
            for (auto v : p) {
                if (count >= 2 && dir(p[count - 2], p[count - 1]) == dir(p[count - 1], v))
                    --count;
                p[count++] = v;
            }
            p.resize(count);
        }
        if (p.size() < 2 || !(p.front() == a && p.back() == b) || !(dir(a, p[1]) & sp.dirs) ||
            !(dir(b, p[p.size() - 2]) & dp.dirs))
            return invalid();
        bool ok = true;
        for (size_t j = 1; j < p.size(); ++j) {
            auto u = p[j - 1], v = p[j];
            if (!dir(u, v))
                return invalid();
            for (Id id : index.query(box(u, v).pad(cfg.buffer))) {
                auto rect = blocks.at(id).rect.pad(cfg.buffer);
                if (!crosses(u, v, rect))
                    continue;
                if ((id == c.src && j == 1 && canExit(u, v, src.rect, aa)) ||
                    (id == c.dst && j + 1 == p.size() && canExit(v, u, dst.rect, bb)))
                    continue;
                if (method != RouteMethod::Native)
                    return false;
                required.insert(id);
                ok = false;
            }
        }
        if (!ok) {
            out.outcome = Outcome::Collision;
            if (profile)
                ++profile->collisionAttempts;
            return false;
        }
        out.status = Status::Valid;
        out.outcome = Outcome::None;
        out.method = method;
        out.points = std::move(p);
        return true;
    };
    if (cfg.fastPaths && !(settled && cfg.settleNudge)) {
        auto it = routes.find(c.id);
        if (it != routes.end() && it->second.method != RouteMethod::None) {
            const auto &p = it->second.points;
            double dx = a.x - p.front().x, dy = a.y - p.front().y;
            if (dx == b.x - p.back().x && dy == b.y - p.back().y) {
                auto moved = p;
                for (auto &v : moved)
                    v = {v.x + dx, v.y + dy};
                moved.front() = a;
                moved.back() = b;
                if (accept(std::move(moved), RouteMethod::Reuse))
                    return out;
            }
            if (p.size() >= 3) {
                auto repair = [&](Point end, Point oldEnd, Point bend) {
                    return end == oldEnd ? bend : oldEnd.y == bend.y ? Point{bend.x, end.y} : Point{end.x, bend.y};
                };
                auto first = repair(a, p.front(), p[1]);
                auto last = repair(b, p.back(), p[p.size() - 2]);
                std::vector<Point> patch{a, first};
                if (p.size() == 3)
                    patch.push_back(p[1]);
                else
                    patch.insert(patch.end(), p.begin() + 2, p.end() - 2);
                patch.push_back(last);
                patch.push_back(b);
                if (accept(std::move(patch), RouteMethod::Reuse))
                    return out;
            }
        }
        if (accept(elbow(a, b, true), RouteMethod::Simple) ||
            (a.x != b.x && a.y != b.y && accept(elbow(a, b, false), RouteMethod::Simple)))
            return out;
        if (a.y != b.y && (sp.dirs & (Left | Right)) && (dp.dirs & (Left | Right)))
            for (double x : {(a.x + b.x) * .5, std::clamp((a.x + b.x) * .5, aa.l, aa.r),
                             std::clamp((a.x + b.x) * .5, bb.l, bb.r)})
                if (accept({a, {x, a.y}, {x, b.y}, b}, RouteMethod::Simple))
                    return out;
        if (a.x != b.x && (sp.dirs & (Up | Down)) && (dp.dirs & (Up | Down)))
            for (double y : {(a.y + b.y) * .5, std::clamp((a.y + b.y) * .5, aa.t, aa.b),
                             std::clamp((a.y + b.y) * .5, bb.t, bb.b)})
                if (accept({a, {a.x, y}, {b.x, y}, b}, RouteMethod::Simple))
                    return out;
    }
    auto bounds = box(a, b);
    for (unsigned attempt = 0; attempt <= cfg.retries && required.size() <= cfg.budget; ++attempt) {
        std::vector<const Block *> obs;
        {
        Stopwatch clock(profile ? &profile->queryMs : nullptr);
            auto ids = index.query(bounds.pad(std::ldexp(cfg.padding, attempt) + cfg.buffer));
            ids.insert(ids.end(), required.begin(), required.end());
            std::sort(ids.begin(), ids.end());
            ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
            if (ids.size() > cfg.budget) { // Budget limits native work, never the collision check below.
                if (profile)
                    ++profile->cappedAttempts;
                double vx = b.x - a.x, vy = b.y - a.y, den = vx * vx + vy * vy;
                auto priority = [&](Id id) {
                    auto r = blocks.at(id).rect;
                    double x = (r.l + r.r) * .5 - a.x, y = (r.t + r.b) * .5 - a.y,
                           k = std::clamp((x * vx + y * vy) / den, 0., 1.);
                    return std::make_tuple(!required.count(id), std::hypot(x - k * vx, y - k * vy), id);
                };
                // Compute geometric keys once, not on every comparison.
                std::vector<std::tuple<bool, double, Id>> keys;
                keys.reserve(ids.size());
                for (Id id : ids)
                    keys.push_back(priority(id));
                std::nth_element(keys.begin(), keys.begin() + cfg.budget, keys.end());
                ids.clear();
                for (unsigned i = 0; i < cfg.budget; ++i)
                    ids.push_back(std::get<2>(keys[i]));
                std::sort(ids.begin(), ids.end());
            }
            for (Id id : ids)
                obs.push_back(&blocks.at(id));
        }
        out.window = std::max<unsigned>(out.window, obs.size());
        out.retries = attempt;
        ++out.calls;
        if (accept(solver->solve({cfg, c, obs, settled && cfg.settleNudge, profile}), RouteMethod::Native))
            return out;
    }
    if (required.size() > cfg.budget)
        out.outcome = Outcome::ObstacleBudget;
    return out;
}
std::vector<Id> LocalOrthogonalRouter::rerouteAffected(const std::vector<Id> &moved, bool settled) {
    auto ids = dirty;
    for (Id id : moved) {
        blocks.at(id);
        auto it = adjacent.find(id);
        if (it != adjacent.end())
            ids.insert(it->second.begin(), it->second.end());
    }
    for (Id id : ids)
        publish(id, solveOne(conns.at(id), settled));
    return {ids.begin(), ids.end()};
}
void LocalOrthogonalRouter::routeAllParallel(unsigned threads) {
    std::vector<Id> ids;
    for (auto &e : conns)
        ids.push_back(e.first);
    if (ids.empty())
        return;
    std::sort(ids.begin(), ids.end());
    std::vector<Route> result(ids.size());
    std::atomic<size_t> next{0};
    auto work = [&] {
        for (;;) {
            auto i = next.fetch_add(1, std::memory_order_relaxed);
            if (i >= ids.size())
                return;
            result[i] = solveOne(std::as_const(conns).at(ids[i]), false);
        }
    };
    if (!threads)
        threads = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::future<void>> workers;
    for (size_t i = 1; i < std::min<size_t>(threads, ids.size()); ++i)
        workers.push_back(std::async(std::launch::async, work));
    work();
    for (auto &w : workers)
        w.get();
    for (size_t i = 0; i < ids.size(); ++i)
        publish(ids[i], std::move(result[i]));
}
void LocalOrthogonalRouter::exportRegionToSVG(const std::string &path, Rect v,
                                              const std::vector<Id> &highlight) const {
    checkBox(v);
    std::ofstream f(path);
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.imbue(std::locale::classic());
    f.precision(15);
    f << "<svg xmlns='http://www.w3.org/2000/svg' viewBox='" << v.l << ' ' << v.t << ' ' << v.r - v.l << ' '
      << v.b - v.t << "'>\n";
    for (Id id : index.query(v)) {
        auto &b = blocks.at(id);
        auto r = b.rect;
        f << "<rect x='" << r.l << "' y='" << r.t << "' width='" << r.r - r.l << "' height='" << r.b - r.t
          << "' fill='lightgray'/>\n";
        for (auto pin : b.pins) {
            auto p = position(r, pin);
            f << "<circle cx='" << p.x << "' cy='" << p.y << "' r='3'/>\n";
        }
    }
    std::set<Id> hi(highlight.begin(), highlight.end());
    for (auto &e : routes) {
        auto p = e.second.status == Status::Pending ? preview(e.first) : e.second.points;
        bool visible = false;
        for (size_t i = 1; i < p.size(); ++i)
            visible |= box(p[i - 1], p[i]).intersects(v);
        if (!visible)
            continue;
        f << "<polyline data-id='" << e.first << "' fill='none' stroke='"
          << (e.second.status != Status::Valid ? "red"
              : hi.count(e.first)              ? "blue"
                                               : "#667")
          << (e.second.status == Status::Pending ? "' stroke-dasharray='4 3" : "") << "' points='";
        for (auto a : p)
            f << a.x << ',' << a.y << ' ';
        f << "'/>\n";
    }
    f << "</svg>\n";
}
} // namespace localroute
