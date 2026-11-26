#include "localroute/async_local_router.h"

#include <utility>

namespace localroute {

AsyncLocalRouter::AsyncLocalRouter(LocalOrthogonalRouter initial)
    : model_(std::move(initial)) {
  LocalOrthogonalRouter snapshot(model_.cfg_, model_.solver_);
  snapshot.index_ = model_.index_;
  snapshot.blocks_ = model_.blocks_;
  snapshot.conns_ = model_.conns_;
  snapshot.adjacent_ = model_.adjacent_;
  snapshot.routes_ = model_.routes_;
  worker_ = std::thread(
      [this, s = std::move(snapshot)]() mutable { Run(std::move(s)); });
}

AsyncLocalRouter::~AsyncLocalRouter() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
  }
  wake_.notify_one();
  worker_.join();
}

void AsyncLocalRouter::Refresh() {
  ++generation_;
  wake_.notify_one();
}

void AsyncLocalRouter::StartAll() {
  std::set<Id> all;
  for (const auto& entry : model_.conns_) {
    model_.Mark(entry.first);
    all.insert(entry.first);
  }
  model_.dirty_.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.merge(all);
  Refresh();
}

void AsyncLocalRouter::StartDirty() {
  if (model_.dirty_.empty()) {
    return;
  }
  auto affected = std::move(model_.dirty_);
  model_.dirty_.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.merge(affected);
  Refresh();
}

void AsyncLocalRouter::Move(const std::vector<std::pair<Id, Rect>>& moves) {
  if (moves.empty()) {
    return;
  }
  model_.MoveBlocksBatch(moves);
  auto affected = std::move(model_.dirty_);
  model_.dirty_.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& m : moves) {
    edits_[m.first] = m.second;
  }
  gesture_.insert(affected.begin(), affected.end());
  for (Id id : affected) {
    nudged_.erase(id);
  }
  pending_.merge(affected);
  Refresh();
}

void AsyncLocalRouter::Prioritize(Rect viewport,
                                  const std::vector<Id>& selection) {
  auto visible = model_.index_.Query(viewport);
  visible.insert(visible.end(), selection.begin(), selection.end());
  std::set<Id> ids;
  for (Id id : visible) {
    const auto it = model_.adjacent_.find(id);
    if (it != model_.adjacent_.end()) {
      ids.insert(it->second.begin(), it->second.end());
    }
  }
  if (model_.cfg_.reroute_third_party_blocked) {
    for (std::uint64_t key : model_.segments_.Query(viewport)) {
      ids.insert(static_cast<Id>(key >> 32));
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (ids != priority_) {
    priority_ = std::move(ids);
    Refresh();
  }
}

void AsyncLocalRouter::Settle() {
  if (gesture_.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_.cfg_.settle_nudge) {
    for (Id id : gesture_) {
      model_.Mark(id);
    }
    nudged_.insert(gesture_.begin(), gesture_.end());
    pending_.merge(gesture_);
  }
  gesture_.clear();
  model_.dirty_.clear();
  Refresh();
}

bool AsyncLocalRouter::MoveRouteSegment(Id conn_id, std::size_t seg_index,
                                        double new_coord) {
  if (!model_.MoveRouteSegment(conn_id, seg_index, new_coord)) {
    return false;
  }
  Route updated = model_.route(conn_id);
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.erase(conn_id);
  gesture_.erase(conn_id);
  nudged_.erase(conn_id);
  dirty_for_nudge_.erase(conn_id);
  route_overrides_[conn_id] = std::move(updated);
  Refresh();
  return true;
}

std::vector<Id> AsyncLocalRouter::AutoRouteConnectors(
    const std::vector<Id>& conn_ids) {
  const std::vector<Id> changed = model_.AutoRouteConnectors(conn_ids);
  if (changed.empty()) {
    return {};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (Id cid : changed) {
    pending_.erase(cid);
    gesture_.erase(cid);
    nudged_.erase(cid);
    dirty_for_nudge_.erase(cid);
    const auto rit = model_.routes_.find(cid);
    if (rit != model_.routes_.end()) {
      route_overrides_[cid] = rit->second;
    }
  }
  Refresh();
  return changed;
}

void AsyncLocalRouter::set_reroute_third_party_blocked(bool enabled) {
  model_.set_reroute_third_party_blocked(enabled);
  std::lock_guard<std::mutex> lock(mutex_);
  has_config_update_ = true;
  pending_reroute_third_party_ = enabled;
  Refresh();
}

void AsyncLocalRouter::set_enable_channel_nudging(bool enabled) {
  model_.set_enable_channel_nudging(enabled);
  std::lock_guard<std::mutex> lock(mutex_);
  has_nudging_update_ = true;
  pending_enable_channel_nudging_ = enabled;
  dirty_for_nudge_.clear();
  Refresh();
}

std::size_t AsyncLocalRouter::Poll(std::size_t max_results,
                                   std::vector<Id>* updated_ids) {
  std::deque<std::pair<Id, Route>> batch;
  bool wave_finished = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) {
      std::rethrow_exception(failure_);
    }
    if (ready_generation_ != generation_) {
      return 0;
    }
    while (!ready_.empty() && batch.size() < max_results) {
      pending_.erase(ready_.front().first);
      nudged_.erase(ready_.front().first);
      batch.push_back(std::move(ready_.front()));
      ready_.pop_front();
    }
    wave_finished = pending_.empty() && ready_.empty();
  }
  wake_.notify_one();
  for (auto& entry : batch) {
    if (updated_ids) {
      updated_ids->push_back(entry.first);
    }
    dirty_for_nudge_.insert(entry.first);
    model_.Publish(entry.first, std::move(entry.second));
  }
  if (wave_finished && model_.cfg_.enable_channel_nudging &&
      !dirty_for_nudge_.empty()) {
    const std::vector<Id> seeds(dirty_for_nudge_.begin(),
                                dirty_for_nudge_.end());
    dirty_for_nudge_.clear();
    const bool full_graph = seeds.size() > 256;
    const std::vector<Id> nudged =
        model_.ApplyChannelNudging(seeds, full_graph);
    if (updated_ids) {
      updated_ids->insert(updated_ids->end(), nudged.begin(), nudged.end());
    }
  }
  return batch.size();
}

std::size_t AsyncLocalRouter::PendingCount() {
  std::lock_guard<std::mutex> lock(mutex_);
  return pending_.size();
}

void AsyncLocalRouter::Run(LocalOrthogonalRouter snapshot) {
  try {
    std::size_t version = 0;
    for (;;) {
      std::vector<std::pair<Id, bool>> ids;
      std::unordered_map<Id, Rect> changes;
      std::unordered_map<Id, Route> route_changes;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [&] { return stopped_ || version != generation_; });
        if (stopped_) {
          return;
        }
        version = generation_;
        changes.swap(edits_);
        route_changes.swap(route_overrides_);
        if (has_config_update_) {
          snapshot.cfg_.reroute_third_party_blocked =
              pending_reroute_third_party_;
          has_config_update_ = false;
        }
        if (has_nudging_update_) {
          snapshot.cfg_.enable_channel_nudging =
              pending_enable_channel_nudging_;
          has_nudging_update_ = false;
        }
        for (Id id : priority_) {
          if (pending_.count(id)) {
            ids.emplace_back(id, nudged_.count(id) != 0);
          }
        }
        for (Id id : pending_) {
          if (!priority_.count(id)) {
            ids.emplace_back(id, nudged_.count(id) != 0);
          }
        }
        ready_.clear();
        ready_generation_ = version;
      }
      for (const auto& entry : changes) {
        snapshot.index_.Update(entry.first, entry.second);
        snapshot.blocks_.at(entry.first).rect = entry.second;
      }
      for (auto& entry : route_changes) {
        snapshot.Publish(entry.first, std::move(entry.second));
      }
      for (const auto& item : ids) {
        {
          std::unique_lock<std::mutex> lock(mutex_);
          wake_.wait(lock, [&] {
            return stopped_ || version != generation_ || ready_.size() < 128;
          });
          if (stopped_ || version != generation_) {
            break;
          }
        }
        const Id id = item.first;
        Route route = snapshot.SolveOne(snapshot.conns_.at(id), item.second);
        snapshot.routes_[id] = route;
        std::lock_guard<std::mutex> lock(mutex_);
        if (version == generation_) {
          ready_.emplace_back(id, std::move(route));
        }
      }
    }
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    failure_ = std::current_exception();
  }
}

}  // namespace localroute
