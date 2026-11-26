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
// Configure topology before construction; recreate the session for topology
// edits.
class AsyncLocalRouter {
 public:
  explicit AsyncLocalRouter(LocalOrthogonalRouter initial);
  ~AsyncLocalRouter();

  AsyncLocalRouter(const AsyncLocalRouter&) = delete;
  AsyncLocalRouter& operator=(const AsyncLocalRouter&) = delete;

  void StartAll();
  void StartDirty();
  void Move(const std::vector<std::pair<Id, Rect>>& moves);
  void Prioritize(Rect viewport, const std::vector<Id>& selection = {});
  void Settle();
  void set_reroute_third_party_blocked(bool enabled);
  void set_enable_channel_nudging(bool enabled);
  std::size_t Poll(std::size_t max_results = 64,
                   std::vector<Id>* updated_ids = nullptr);
  std::size_t PendingCount();
  const LocalOrthogonalRouter& view() const { return model_; }

 private:
  void Run(LocalOrthogonalRouter snapshot);
  void Refresh();  // Must be called with mutex_ held.

  LocalOrthogonalRouter model_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::set<Id> pending_;
  std::set<Id> gesture_;
  std::set<Id> priority_;
  std::set<Id> nudged_;
  std::unordered_map<Id, Rect> edits_;
  std::deque<std::pair<Id, Route>> ready_;
  std::size_t generation_ = 0;
  std::size_t ready_generation_ = 0;
  bool stopped_ = false;
  bool has_config_update_ = false;
  bool pending_reroute_third_party_ = false;
  bool has_nudging_update_ = false;
  bool pending_enable_channel_nudging_ = true;
  std::set<Id> dirty_for_nudge_;
  std::exception_ptr failure_;
  std::thread worker_;
};

}  // namespace localroute

#endif  // LOCALROUTE_INCLUDE_LOCALROUTE_ASYNC_LOCAL_ROUTER_H_
