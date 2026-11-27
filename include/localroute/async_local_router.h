#ifndef LOCALROUTE_INCLUDE_LOCALROUTE_ASYNC_LOCAL_ROUTER_H_
#define LOCALROUTE_INCLUDE_LOCALROUTE_ASYNC_LOCAL_ROUTER_H_

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "localroute/local_orthogonal_router.h"

namespace localroute {

// Owner thread edits and polls; a private worker never touches displayed state.
// Supports incremental block/connector/pin mutations via a strict FIFO delta
// queue synchronized with block moves and route overrides.
class AsyncLocalRouter {
 public:
  explicit AsyncLocalRouter(LocalOrthogonalRouter initial);
  ~AsyncLocalRouter();

  AsyncLocalRouter(const AsyncLocalRouter&) = delete;
  AsyncLocalRouter& operator=(const AsyncLocalRouter&) = delete;

  void StartAll();
  void StartDirty();
  void AddBlock(Block block);
  void RemoveBlock(Id id);
  void AddConnector(Conn conn);
  void RemoveConnector(Id id);
  std::vector<Id> UpdateBlockPins(Id block_id, Rect new_rect,
                                  std::vector<Pin> new_pins);
  void RestoreRoute(Id conn_id, Route route);
  void Move(const std::vector<std::pair<Id, Rect>>& moves);
  void Prioritize(Rect viewport, const std::vector<Id>& selection = {});
  void Settle();
  bool MoveRouteSegment(Id conn_id, std::size_t seg_index, double new_coord,
                        std::vector<Id>* changed_out = nullptr);
  std::vector<Id> AutoRouteConnectors(const std::vector<Id>& conn_ids);
  void set_reroute_third_party_blocked(bool enabled);
  void set_enable_channel_nudging(bool enabled);
  std::size_t Poll(std::size_t max_results = 64,
                   std::vector<Id>* updated_ids = nullptr);
  std::size_t PendingCount();
  const LocalOrthogonalRouter& view() const { return model_; }

 private:
  struct PendingOp {
    enum class Kind {
      kMoveBatch,
      kAddBlock,
      kRemoveBlock,
      kAddConnector,
      kRemoveConnector,
      kUpdateBlockPins,
      kSetRoute,
    };
    Kind kind = Kind::kMoveBatch;
    Id id = 0;
    Rect rect{};
    Block block;
    Conn conn{};
    std::vector<Pin> pins;
    std::vector<std::pair<Id, Rect>> moves;
    Route route;
  };

  void Run(LocalOrthogonalRouter snapshot);
  void Refresh();                    // Must be called with mutex_ held.
  void FlushCoalescedEditsLocked();  // Must be called with mutex_ held.

  LocalOrthogonalRouter model_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::set<Id> pending_;
  std::set<Id> gesture_;
  std::set<Id> priority_;
  std::set<Id> nudged_;
  std::unordered_map<Id, Rect> edits_;
  std::deque<PendingOp> pending_ops_;
  std::deque<std::pair<Id, Route>> ready_;
  std::size_t generation_ = 0;
  std::size_t ready_generation_ = 0;
  bool stopped_ = false;
  bool has_config_update_ = false;
  bool pending_reroute_third_party_ = false;
  bool has_nudging_update_ = false;
  bool pending_enable_channel_nudging_ = true;
  bool full_wave_pending_ = false;
  bool priority_nudged_ = false;
  std::set<Id> dirty_for_nudge_;
  std::set<Id> dirty_priority_for_nudge_;
  std::exception_ptr failure_;
  std::thread worker_;
};

}  // namespace localroute

#endif  // LOCALROUTE_INCLUDE_LOCALROUTE_ASYNC_LOCAL_ROUTER_H_
