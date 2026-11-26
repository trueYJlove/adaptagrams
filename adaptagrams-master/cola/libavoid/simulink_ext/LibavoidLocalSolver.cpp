#include "ILocalRouteSolver.h"
#include "libavoid/libavoid.h"
#include <chrono>
namespace localroute {
namespace {
struct LibavoidLocalSolver final : ILocalRouteSolver {
    const char *name() const override { return "LibavoidLocalSolver"; }
    std::vector<Point> solve(const Request &q) const override {
        using Clock = std::chrono::steady_clock;
        auto start = q.profile ? Clock::now() : Clock::time_point();
        Avoid::TransactionTiming timing;
        Avoid::Router r(Avoid::OrthogonalRouting);
        if (q.profile)
            r.setTransactionTiming(&timing);
        r.setRoutingParameter(Avoid::shapeBufferDistance, q.cfg.buffer);
        r.setRoutingParameter(Avoid::segmentPenalty, q.cfg.penalty);
        r.setRoutingParameter(Avoid::idealNudgingDistance, q.cfg.nudgeDistance);
        r.setRoutingOption(Avoid::performUnifyingNudgingPreprocessingStep, false);
        r.setRoutingOption(Avoid::improveHyperedgeRoutesMovingJunctions, false);
        r.setRoutingOption(Avoid::nudgeSharedPathsWithCommonEndPoint, false);
        r.setRoutingOption(Avoid::enableOrthogonalPostprocessing, q.nudge);
        Avoid::ShapeRef *s = nullptr, *d = nullptr;
        for (auto b : q.obstacles) {
            auto a = b->rect;
            Avoid::Rectangle rect({a.l, a.t}, {a.r, a.b});
            auto sh = new Avoid::ShapeRef(&r, rect);
            if (b->id == q.conn.src)
                s = sh;
            if (b->id == q.conn.dst)
                d = sh;
            for (auto p : b->pins)
                if ((s == sh && p.id == q.conn.sp) || (d == sh && p.id == q.conn.dp)) {
                    auto pin = new Avoid::ShapeConnectionPin(sh, p.id, p.x, p.y, true, 0, p.dirs);
                    pin->setExclusive(false);
                }
        }
        if (!s || !d)
            return {};
        auto c = new Avoid::ConnRef(&r, Avoid::ConnEnd(s, q.conn.sp), Avoid::ConnEnd(d, q.conn.dp));
        auto elapsed = [&] { return std::chrono::duration<double, std::milli>(Clock::now() - start).count(); };
        if (q.profile) {
            q.profile->setupMs += elapsed();
            start = Clock::now();
        }
        r.processTransaction();
        if (q.profile) {
            q.profile->graphMs += timing.graphMs;
            q.profile->searchMs += timing.searchMs;
            q.profile->nativeOtherMs += std::max(0., elapsed() - timing.graphMs - timing.searchMs);
        }
        std::vector<Point> out;
        for (auto p : c->displayRoute().ps)
            out.push_back({p.x, p.y});
        return out;
    }
};
} // namespace
std::shared_ptr<const ILocalRouteSolver> libavoidSolver() { return std::make_shared<LibavoidLocalSolver>(); }
} // namespace localroute
