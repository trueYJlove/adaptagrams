#pragma once
#include "LocalOrthogonalRouter.h"
#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>
#include <thread>

namespace localroute {
// Owner thread edits and polls; a private worker never touches displayed state.
// Configure topology before construction; destroy/recreate the session for topology edits.
class AsyncLocalRouter {
    LocalOrthogonalRouter model;
    std::mutex mutex;
    std::condition_variable wake;
    std::set<Id> pending, gesture, priority, nudged;
    std::unordered_map<Id, Rect> edits;
    std::deque<std::pair<Id, Route>> ready;
    std::size_t generation = 0, readyGeneration = 0;
    bool stopped = false;
    std::exception_ptr failure;
    std::thread worker;
    void run(LocalOrthogonalRouter);
    void refresh(); // under mutex

  public:
    explicit AsyncLocalRouter(LocalOrthogonalRouter);
    ~AsyncLocalRouter();
    void startAll();
    void move(const std::vector<std::pair<Id, Rect>> &);
    void prioritize(Rect viewport, const std::vector<Id> &selection = {});
    void settle();
    std::size_t poll(std::size_t maxResults = 64);
    std::size_t pendingCount();
    const LocalOrthogonalRouter &view() const { return model; }
};
} // namespace localroute
