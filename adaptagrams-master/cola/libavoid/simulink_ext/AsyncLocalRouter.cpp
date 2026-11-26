#include "AsyncLocalRouter.h"

namespace localroute {
AsyncLocalRouter::AsyncLocalRouter(LocalOrthogonalRouter initial) : model(std::move(initial)) {
    LocalOrthogonalRouter snapshot(model.cfg, model.solver);
    snapshot.index = model.index;
    snapshot.blocks = model.blocks;
    snapshot.conns = model.conns;
    snapshot.routes = model.routes;
    worker = std::thread([this, snapshot = std::move(snapshot)]() mutable { run(std::move(snapshot)); });
}
AsyncLocalRouter::~AsyncLocalRouter() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        stopped = true;
    }
    wake.notify_one();
    worker.join();
}
void AsyncLocalRouter::refresh() {
    ++generation;
    wake.notify_one();
}
void AsyncLocalRouter::startAll() {
    // ponytail: one O(lines) queue setup on open; no full model copy on every drag.
    std::set<Id> all;
    for (auto &e : model.conns) {
        model.mark(e.first);
        all.insert(e.first);
    }
    model.dirty.clear();
    std::lock_guard<std::mutex> lock(mutex);
    pending.merge(all);
    refresh();
}
void AsyncLocalRouter::move(const std::vector<std::pair<Id, Rect>> &moves) {
    if (moves.empty())
        return;
    model.moveBlocksBatch(moves);
    // This set only holds newly invalidated lines in an async session.
    auto affected = std::move(model.dirty);
    model.dirty.clear();
    std::lock_guard<std::mutex> lock(mutex);
    for (auto &m : moves)
        edits[m.first] = m.second;
    gesture.insert(affected.begin(), affected.end());
    for (Id id : affected)
        nudged.erase(id);
    pending.merge(affected);
    refresh();
}
void AsyncLocalRouter::prioritize(Rect viewport, const std::vector<Id> &selection) {
    auto visible = model.index.query(viewport);
    visible.insert(visible.end(), selection.begin(), selection.end());
    std::set<Id> ids;
    for (Id id : visible) {
        auto it = model.adjacent.find(id);
        if (it != model.adjacent.end())
            ids.insert(it->second.begin(), it->second.end());
    }
    if (model.cfg.rerouteThirdPartyBlocked)
        for (auto key : model.segments.query(viewport))
            ids.insert(Id(key >> 32));
    std::lock_guard<std::mutex> lock(mutex);
    if (ids != priority) {
        priority = std::move(ids);
        refresh();
    }
}
void AsyncLocalRouter::settle() {
    if (gesture.empty())
        return;
    std::lock_guard<std::mutex> lock(mutex);
    if (model.cfg.settleNudge) {
        for (Id id : gesture)
            model.mark(id);
        nudged.insert(gesture.begin(), gesture.end());
        pending.merge(gesture);
    }
    gesture.clear();
    model.dirty.clear();
    refresh();
}
std::size_t AsyncLocalRouter::poll(std::size_t maxResults) {
    std::deque<std::pair<Id, Route>> batch;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (failure)
            std::rethrow_exception(failure);
        if (readyGeneration != generation)
            return 0;
        while (!ready.empty() && batch.size() < maxResults) {
            pending.erase(ready.front().first);
            nudged.erase(ready.front().first);
            batch.push_back(std::move(ready.front()));
            ready.pop_front();
        }
    }
    wake.notify_one();
    for (auto &e : batch)
        model.publish(e.first, std::move(e.second));
    return batch.size();
}
std::size_t AsyncLocalRouter::pendingCount() {
    std::lock_guard<std::mutex> lock(mutex);
    return pending.size();
}
void AsyncLocalRouter::run(LocalOrthogonalRouter snapshot) {
    try {
        std::size_t version = 0;
        for (;;) {
            std::vector<std::pair<Id, bool>> ids;
            std::unordered_map<Id, Rect> changes;
            {
                std::unique_lock<std::mutex> lock(mutex);
                wake.wait(lock, [&] { return stopped || version != generation; });
                if (stopped)
                    return;
                version = generation;
                changes.swap(edits);
                // Visible/selected connectors first. IDs are deterministic within each group.
                for (Id id : priority)
                    if (pending.count(id))
                        ids.emplace_back(id, nudged.count(id) != 0);
                for (Id id : pending)
                    if (!priority.count(id))
                        ids.emplace_back(id, nudged.count(id) != 0);
                ready.clear();
                readyGeneration = version;
            }
            for (auto &e : changes) {
                snapshot.index.update(e.first, e.second);
                snapshot.blocks.at(e.first).rect = e.second;
            }
            for (auto item : ids) {
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    wake.wait(lock, [&] { return stopped || version != generation || ready.size() < 128; });
                    if (stopped || version != generation)
                        break;
                }
                Id id = item.first;
                auto route = snapshot.solveOne(snapshot.conns.at(id), item.second);
                snapshot.routes[id] = route;
                std::lock_guard<std::mutex> lock(mutex);
                if (version == generation)
                    ready.emplace_back(id, std::move(route));
            }
        }
    } catch (...) {
        std::lock_guard<std::mutex> lock(mutex);
        failure = std::current_exception();
    }
}
} // namespace localroute
