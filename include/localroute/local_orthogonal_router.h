#ifndef LOCALROUTE_INCLUDE_LOCALROUTE_LOCAL_ORTHOGONAL_ROUTER_H_
#define LOCALROUTE_INCLUDE_LOCALROUTE_LOCAL_ORTHOGONAL_ROUTER_H_

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "localroute/spatial_grid_index.h"

namespace localroute {

// Synchronous API: the caller must not access this model concurrently with a
// mutation or batch operation.
class LocalOrthogonalRouter {
 public:
  explicit LocalOrthogonalRouter(
      Config cfg = {}, std::shared_ptr<const LocalRouteSolver> solver = {});

  void AddBlock(Block block);
  void RemoveBlock(Id id);
  void AddConnector(Conn conn);
  void RemoveConnector(Id id);
  void MoveBlocksBatch(const std::vector<std::pair<Id, Rect>>& moves);
  void MoveBlock(Id id, double dx, double dy);
  std::vector<Id> RerouteAffected(const std::vector<Id>& moved,
                                  bool settled = false);
  void RouteAllParallel(unsigned threads = 0);
  void RouteConnector(Id id);
  void MarkConnector(Id id);
  LocalOrthogonalRouter Clone() const { return *this; }
  std::vector<Point> Preview(Id id) const;
  void set_reroute_third_party_blocked(bool enabled);
  void set_enable_channel_nudging(bool enabled);

  const Config& config() const { return cfg_; }
  std::vector<Id> QueryBlocks(Rect area) const { return index_.Query(area); }
  const std::unordered_map<Id, Block>& all_blocks() const { return blocks_; }
  const std::unordered_map<Id, Conn>& all_conns() const { return conns_; }
  const std::unordered_map<Id, Route>& all_routes() const { return routes_; }
  const std::vector<Id>& adjacent_conns(Id block_id) const {
    static const std::vector<Id> kEmpty;
    const auto it = adjacent_.find(block_id);
    return it == adjacent_.end() ? kEmpty : it->second;
  }
  const Route& route(Id id) const { return routes_.at(id); }
  const Block& block(Id id) const { return blocks_.at(id); }
  const char* solver_name() const { return solver_->name(); }

  void ExportRegionToSvg(const std::string& path, Rect viewport,
                         const std::vector<Id>& highlight = {}) const;

 private:
  Route SolveOne(const Conn& conn, bool settled) const;
  void Mark(Id id);
  void InvalidateObstacle(Id block_id, Rect rect);
  void ClearSegments(Id id);
  void Publish(Id id, Route route);
  bool ValidateAndSimplifyRoute(const Conn& conn,
                                std::vector<Point>* points) const;
  std::vector<Id> ApplyChannelNudging(const std::vector<Id>& mutable_conns,
                                      bool full_graph);

  friend class AsyncLocalRouter;
  friend class ChannelNudger;

  Config cfg_;
  SpatialGridIndex index_;
  std::shared_ptr<const LocalRouteSolver> solver_;
  std::unordered_map<Id, Block> blocks_;
  std::unordered_map<Id, Conn> conns_;
  std::unordered_map<Id, std::vector<Id>> adjacent_;
  std::unordered_map<Id, Route> routes_;
  SpatialGrid<std::uint64_t> segments_;
  std::set<Id> dirty_;
};

}  // namespace localroute

#endif  // LOCALROUTE_INCLUDE_LOCALROUTE_LOCAL_ORTHOGONAL_ROUTER_H_
