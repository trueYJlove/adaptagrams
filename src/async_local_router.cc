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

void AsyncLocalRouter::FlushCoalescedEditsLocked() {
  if (edits_.empty()) {
    return;
  }
  PendingOp op;
  op.kind = PendingOp::Kind::kMoveBatch;
  op.moves.reserve(edits_.size());
  for (const auto& kv : edits_) {
    op.moves.push_back(kv);
  }
  pending_ops_.push_back(std::move(op));
  edits_.clear();
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
  priority_nudged_ = false;
  dirty_priority_for_nudge_.clear();
  Refresh();
}

void AsyncLocalRouter::StartDirty() {
  if (model_.dirty_.empty()) {
    return;
  }
  auto affected = std::move(model_.dirty_);
  model_.dirty_.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  for (Id id : affected) {
    if (priority_.count(id)) {
      priority_nudged_ = false;
      break;
    }
  }
  pending_.merge(affected);
  Refresh();
}

void AsyncLocalRouter::AddBlock(Block block) {
  Block copy = block;
  model_.AddBlock(std::move(block));
  auto affected = std::move(model_.dirty_);
  model_.dirty_.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  FlushCoalescedEditsLocked();
  PendingOp op;
  op.kind = PendingOp::Kind::kAddBlock;
  op.id = copy.id;
  op.block = std::move(copy);
  pending_ops_.push_back(std::move(op));
  if (!affected.empty()) {
    for (Id id : affected) {
      nudged_.erase(id);
      if (priority_.count(id)) {
        priority_nudged_ = false;
      }
    }
    pending_.merge(affected);
  }
  Refresh();
}

void AsyncLocalRouter::RemoveBlock(Id id) {
  (void)model_.blocks_.at(id);
  const std::vector<Id> removed_conns = model_.adjacent_conns(id);
  model_.RemoveBlock(id);
  std::lock_guard<std::mutex> lock(mutex_);
  FlushCoalescedEditsLocked();
  PendingOp op;
  op.kind = PendingOp::Kind::kRemoveBlock;
  op.id = id;
  pending_ops_.push_back(std::move(op));
  for (Id cid : removed_conns) {
    pending_.erase(cid);
    gesture_.erase(cid);
    priority_.erase(cid);
    nudged_.erase(cid);
    dirty_for_nudge_.erase(cid);
    dirty_priority_for_nudge_.erase(cid);
  }
  Refresh();
}

void AsyncLocalRouter::AddConnector(Conn conn) {
  model_.AddConnector(conn);
  model_.MarkConnector(conn.id);
  model_.dirty_.erase(conn.id);
  std::lock_guard<std::mutex> lock(mutex_);
  FlushCoalescedEditsLocked();
  PendingOp op;
  op.kind = PendingOp::Kind::kAddConnector;
  op.id = conn.id;
  op.conn = conn;
  pending_ops_.push_back(std::move(op));
  pending_.insert(conn.id);
  nudged_.erase(conn.id);
  if (priority_.count(conn.id)) {
    priority_nudged_ = false;
  }
  Refresh();
}

void AsyncLocalRouter::RemoveConnector(Id id) {
  model_.RemoveConnector(id);
  std::lock_guard<std::mutex> lock(mutex_);
  FlushCoalescedEditsLocked();
  PendingOp op;
  op.kind = PendingOp::Kind::kRemoveConnector;
  op.id = id;
  pending_ops_.push_back(std::move(op));
  pending_.erase(id);
  gesture_.erase(id);
  priority_.erase(id);
  nudged_.erase(id);
  dirty_for_nudge_.erase(id);
  dirty_priority_for_nudge_.erase(id);
  Refresh();
}

std::vector<Id> AsyncLocalRouter::UpdateBlockPins(Id block_id, Rect new_rect,
                                                  std::vector<Pin> new_pins) {
  const std::vector<Id> old_adj = model_.adjacent_conns(block_id);
  std::vector<Pin> pins_copy = new_pins;
  std::vector<Id> surviving =
      model_.UpdateBlockPins(block_id, new_rect, std::move(new_pins));
  auto extra_dirty = std::move(model_.dirty_);
  model_.dirty_.clear();

  std::vector<Id> removed_conns;
  for (Id cid : old_adj) {
    if (!model_.conns_.count(cid)) {
      removed_conns.push_back(cid);
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  FlushCoalescedEditsLocked();
  PendingOp op;
  op.kind = PendingOp::Kind::kUpdateBlockPins;
  op.id = block_id;
  op.rect = new_rect;
  op.pins = std::move(pins_copy);
  pending_ops_.push_back(std::move(op));

  for (Id cid : removed_conns) {
    pending_.erase(cid);
    gesture_.erase(cid);
    priority_.erase(cid);
    nudged_.erase(cid);
    dirty_for_nudge_.erase(cid);
    dirty_priority_for_nudge_.erase(cid);
  }
  for (Id cid : surviving) {
    pending_.insert(cid);
    nudged_.erase(cid);
    if (priority_.count(cid)) {
      priority_nudged_ = false;
    }
  }
  for (Id cid : extra_dirty) {
    pending_.insert(cid);
    nudged_.erase(cid);
    if (priority_.count(cid)) {
      priority_nudged_ = false;
    }
  }
  Refresh();
  return surviving;
}

void AsyncLocalRouter::RestoreRoute(Id conn_id, Route route) {
  Route copy = route;
  model_.RestoreRoute(conn_id, std::move(route));
  std::lock_guard<std::mutex> lock(mutex_);
  FlushCoalescedEditsLocked();
  pending_.erase(conn_id);
  gesture_.erase(conn_id);
  nudged_.erase(conn_id);
  dirty_for_nudge_.erase(conn_id);
  dirty_priority_for_nudge_.erase(conn_id);
  PendingOp op;
  op.kind = PendingOp::Kind::kSetRoute;
  op.id = conn_id;
  op.route = std::move(copy);
  pending_ops_.push_back(std::move(op));
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
    if (priority_.count(id)) {
      priority_nudged_ = false;
    }
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
    priority_nudged_ = false;
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
      if (priority_.count(id)) {
        priority_nudged_ = false;
      }
    }
    nudged_.insert(gesture_.begin(), gesture_.end());
    pending_.merge(gesture_);
  }
  gesture_.clear();
  model_.dirty_.clear();
  Refresh();
}

bool AsyncLocalRouter::MoveRouteSegment(Id conn_id, std::size_t seg_index,
                                        double new_coord,
                                        std::vector<Id>* changed_out) {
  std::vector<Id> changed;
  if (!model_.MoveRouteSegment(conn_id, seg_index, new_coord, &changed)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  FlushCoalescedEditsLocked();
  for (Id cid : changed) {
    pending_.erase(cid);
    gesture_.erase(cid);
    nudged_.erase(cid);
    dirty_for_nudge_.erase(cid);
    dirty_priority_for_nudge_.erase(cid);
    PendingOp op;
    op.kind = PendingOp::Kind::kSetRoute;
    op.id = cid;
    op.route = model_.route(cid);
    pending_ops_.push_back(std::move(op));
  }
  Refresh();
  if (changed_out) {
    *changed_out = std::move(changed);
  }
  return true;
}

std::vector<Id> AsyncLocalRouter::AutoRouteConnectors(
    const std::vector<Id>& conn_ids) {
  const std::vector<Id> changed = model_.AutoRouteConnectors(conn_ids);
  if (changed.empty()) {
    return {};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  FlushCoalescedEditsLocked();
  for (Id cid : changed) {
    pending_.erase(cid);
    gesture_.erase(cid);
    nudged_.erase(cid);
    dirty_for_nudge_.erase(cid);
    dirty_priority_for_nudge_.erase(cid);
    const auto rit = model_.routes_.find(cid);
    if (rit != model_.routes_.end()) {
      PendingOp op;
      op.kind = PendingOp::Kind::kSetRoute;
      op.id = cid;
      op.route = rit->second;
      pending_ops_.push_back(std::move(op));
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
  dirty_priority_for_nudge_.clear();
  Refresh();
}

std::size_t AsyncLocalRouter::Poll(std::size_t max_results,
                                   std::vector<Id>* updated_ids) {
  std::deque<std::pair<Id, Route>> batch;
  std::vector<bool> batch_is_pri;
  bool wave_finished = false;
  bool priority_wave_finished = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) {
      std::rethrow_exception(failure_);
    }
    if (ready_generation_ != generation_) {
      return 0;
    }
    batch_is_pri.reserve(std::min(max_results, ready_.size()));
    while (!ready_.empty() && batch.size() < max_results) {
      const Id cid = ready_.front().first;
      pending_.erase(cid);
      nudged_.erase(cid);
      batch_is_pri.push_back(priority_.count(cid) != 0);
      batch.push_back(std::move(ready_.front()));
      ready_.pop_front();
    }
    wave_finished = pending_.empty() && ready_.empty();
    if (!wave_finished && !priority_nudged_ && !priority_.empty() &&
        !batch.empty()) {
      bool any_pri_pending = false;
      for (Id pid : priority_) {
        if (pending_.count(pid)) {
          any_pri_pending = true;
          break;
        }
      }
      if (!any_pri_pending) {
        priority_wave_finished = true;
        priority_nudged_ = true;
      }
    }
  }
  wake_.notify_one();
  for (std::size_t i = 0; i < batch.size(); ++i) {
    auto& entry = batch[i];
    if (updated_ids) {
      updated_ids->push_back(entry.first);
    }
    dirty_for_nudge_.insert(entry.first);
    if (batch_is_pri[i]) {
      dirty_priority_for_nudge_.insert(entry.first);
    }
    model_.Publish(entry.first, std::move(entry.second));
  }
  if (wave_finished && model_.cfg_.enable_channel_nudging &&
      !dirty_for_nudge_.empty()) {
    const std::vector<Id> seeds(dirty_for_nudge_.begin(),
                                dirty_for_nudge_.end());
    dirty_for_nudge_.clear();
    dirty_priority_for_nudge_.clear();
    const bool full_graph = seeds.size() > 256;
    const std::vector<Id> nudged =
        model_.ApplyChannelNudging(seeds, full_graph);
    if (updated_ids) {
      updated_ids->insert(updated_ids->end(), nudged.begin(), nudged.end());
    }
  } else if (priority_wave_finished && model_.cfg_.enable_channel_nudging &&
             !dirty_priority_for_nudge_.empty()) {
    const std::vector<Id> pri_seeds(dirty_priority_for_nudge_.begin(),
                                    dirty_priority_for_nudge_.end());
    dirty_priority_for_nudge_.clear();
    const std::vector<Id> nudged =
        model_.ApplyChannelNudging(pri_seeds, /*full_graph=*/false);
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
      std::deque<PendingOp> ops;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [&] { return stopped_ || version != generation_; });
        if (stopped_) {
          return;
        }
        version = generation_;
        FlushCoalescedEditsLocked();
        ops.swap(pending_ops_);
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
      for (auto& op : ops) {
        switch (op.kind) {
          case PendingOp::Kind::kMoveBatch: {
            std::vector<std::pair<Id, Rect>> live_moves;
            live_moves.reserve(op.moves.size());
            for (const auto& m : op.moves) {
              if (snapshot.blocks_.count(m.first)) {
                live_moves.push_back(m);
              }
            }
            if (!live_moves.empty()) {
              snapshot.MoveBlocksBatch(live_moves);
              snapshot.dirty_.clear();
            }
            break;
          }
          case PendingOp::Kind::kAddBlock:
            snapshot.AddBlock(std::move(op.block));
            snapshot.dirty_.clear();
            break;
          case PendingOp::Kind::kRemoveBlock:
            if (snapshot.blocks_.count(op.id)) {
              snapshot.RemoveBlock(op.id);
            }
            break;
          case PendingOp::Kind::kAddConnector:
            if (snapshot.blocks_.count(op.conn.src) &&
                snapshot.blocks_.count(op.conn.dst) &&
                !snapshot.conns_.count(op.conn.id)) {
              snapshot.AddConnector(op.conn);
            }
            break;
          case PendingOp::Kind::kRemoveConnector:
            if (snapshot.conns_.count(op.id)) {
              snapshot.RemoveConnector(op.id);
            }
            break;
          case PendingOp::Kind::kUpdateBlockPins:
            if (snapshot.blocks_.count(op.id)) {
              snapshot.UpdateBlockPins(op.id, op.rect, std::move(op.pins));
              snapshot.dirty_.clear();
            }
            break;
          case PendingOp::Kind::kSetRoute:
            if (snapshot.conns_.count(op.id)) {
              snapshot.Publish(op.id, std::move(op.route));
            }
            break;
        }
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
        if (!snapshot.conns_.count(id)) {
          continue;
        }
        Route route = snapshot.SolveOne(snapshot.conns_.at(id), item.second);
        snapshot.Publish(id, route);
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
