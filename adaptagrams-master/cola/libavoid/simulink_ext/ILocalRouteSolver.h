#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>
namespace localroute {
using Id = std::uint32_t;
struct Point {
    double x, y;
};
inline bool operator==(Point a, Point b) { return a.x == b.x && a.y == b.y; }
struct Rect {
    double l, t, r, b;
    Rect pad(double d) const { return {l - d, t - d, r + d, b + d}; }
    bool intersects(Rect a) const { return l <= a.r && a.l <= r && t <= a.b && a.t <= b; }
    bool inside(Point p) const { return l < p.x && p.x < r && t < p.y && p.y < b; }
};
inline Rect box(Point a, Point b) {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::max(a.x, b.x), std::max(a.y, b.y)};
}
enum Direction : unsigned { Up = 1, Down = 2, Left = 4, Right = 8 };
struct Pin {
    Id id;
    double x, y;
    unsigned dirs;
}; // normalized coordinates; shared electrical terminal
struct Block {
    Id id;
    Rect rect;
    std::vector<Pin> pins;
};
struct Conn {
    Id id, src, sp, dst, dp;
};
struct Config {
    double cell = 256, padding = 250, buffer = 8, penalty = 50, nudgeDistance = 6;
    unsigned retries = 3, budget = 120;
    bool settleNudge = false, fastPaths = true, profile = false, rerouteThirdPartyBlocked = false;
};
enum class Status { Valid, DisplayOnly, Failed, Pending };
enum class Outcome { None, CoincidentPins, BuriedPin, InvalidCandidate, Collision, ObstacleBudget };
enum class RouteMethod { None, Reuse, Simple, Native };
struct Profile {
    double queryMs = 0, setupMs = 0, graphMs = 0, searchMs = 0, validationMs = 0, nativeOtherMs = 0;
    unsigned invalidAttempts = 0, collisionAttempts = 0, cappedAttempts = 0;
};
struct Route {
    Status status = Status::Failed;
    std::vector<Point> points;
    unsigned calls = 0, retries = 0, window = 0;
    Outcome outcome = Outcome::None;
    // Pending routes may retain previously validated points and their method for reuse.
    RouteMethod method = RouteMethod::None;
    Profile profile;
};
struct Request {
    const Config &cfg;
    const Conn &conn;
    const std::vector<const Block *> &obstacles;
    bool nudge;
    Profile *profile = nullptr;
};
struct ILocalRouteSolver {
    virtual ~ILocalRouteSolver() = default;
    virtual const char *name() const = 0;
    virtual std::vector<Point> solve(const Request &) const = 0;
};
std::shared_ptr<const ILocalRouteSolver> libavoidSolver();
std::shared_ptr<const ILocalRouteSolver> lazyHananAStarSolver();
} // namespace localroute
