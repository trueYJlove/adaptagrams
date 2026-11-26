#pragma once
#include "SpatialGridIndex.h"
#include <set>
#include <string>
namespace localroute {
// Synchronous API: the caller must not access this model concurrently with a mutation/batch.
class LocalOrthogonalRouter {
    Config cfg;
    SpatialGridIndex index;
    std::shared_ptr<const ILocalRouteSolver> solver;
    std::unordered_map<Id, Block> blocks;
    std::unordered_map<Id, Conn> conns;
    std::unordered_map<Id, std::vector<Id>> adjacent;
    std::unordered_map<Id, Route> routes;
    SpatialGrid<std::uint64_t> segments;
    std::set<Id> dirty;
    Route solveOne(const Conn &, bool) const;
    void mark(Id);
    void invalidateObstacle(Id, Rect);
    void clearSegments(Id);
    void publish(Id, Route);
    friend class AsyncLocalRouter;

  public:
    explicit LocalOrthogonalRouter(Config = {}, std::shared_ptr<const ILocalRouteSolver> = {});
    void addBlock(Block);
    void removeBlock(Id);
    void addConnector(Conn);
    void removeConnector(Id);
    void moveBlocksBatch(const std::vector<std::pair<Id, Rect>> &);
    void moveBlock(Id, double, double);
    std::vector<Id> rerouteAffected(const std::vector<Id> &moved, bool settled = false);
    void routeAllParallel(unsigned threads = 0);
    std::vector<Point> preview(Id id) const;
    const Route &route(Id id) const { return routes.at(id); }
    const Block &block(Id id) const { return blocks.at(id); }
    const char *solverName() const { return solver->name(); }
    void exportRegionToSVG(const std::string &, Rect, const std::vector<Id> &highlight = {}) const;
};
} // namespace localroute
