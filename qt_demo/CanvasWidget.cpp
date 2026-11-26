#include "CanvasWidget.h"
#include "topology_generator.h"
#include <QKeyEvent>
#include <QLineF>
#include <QMouseEvent>
#include <QPainter>
#include <QVector>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
localroute::Point pinPos(const localroute::Block &b, localroute::Id pinId) {
    for (const auto &p : b.pins) {
        if (p.id == pinId) {
            return {p.x == 1.0 ? b.rect.r : b.rect.l + p.x * (b.rect.r - b.rect.l),
                    p.y == 1.0 ? b.rect.b : b.rect.t + p.y * (b.rect.b - b.rect.t)};
        }
    }
    return {(b.rect.l + b.rect.r) * 0.5, (b.rect.t + b.rect.b) * 0.5};
}

localroute::Rect pointsBounds(const std::vector<localroute::Point> &pts) {
    if (pts.empty())
        return {0, 0, 0, 0};
    double l = pts.front().x, r = l, t = pts.front().y, b = t;
    for (const auto &p : pts) {
        l = std::min(l, p.x);
        r = std::max(r, p.x);
        t = std::min(t, p.y);
        b = std::max(b, p.y);
    }
    return {l, t, r, b};
}
} // namespace

CanvasWidget::CanvasWidget(QWidget *parent) : QWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    setAutoFillBackground(false);
    fpsClock.start();

    asyncTimer.setInterval(16);
    connect(&asyncTimer, &QTimer::timeout, this, &CanvasWidget::onAsyncTick);
}

CanvasWidget::~CanvasWidget() {
    asyncTimer.stop();
    asyncRouter.reset();
    syncRouter.reset();
}

const localroute::LocalOrthogonalRouter &CanvasWidget::activeView() const {
    return (mode == RouteMode::Async && asyncRouter) ? asyncRouter->view() : *syncRouter;
}

localroute::LocalOrthogonalRouter CanvasWidget::extractModelCopy() const {
    const auto &view = activeView();
    localroute::Config cfg = view.config();
    cfg.rerouteThirdPartyBlocked = thirdPartyReroute;
    localroute::LocalOrthogonalRouter copy(cfg);
    for (const auto &kv : view.allBlocks())
        copy.addBlock(kv.second);
    for (const auto &kv : view.allConns())
        copy.addConnector(kv.second);
    return copy;
}

void CanvasWidget::loadTopology(unsigned blockCount) {
    asyncTimer.stop();
    asyncRouter.reset();
    syncRouter.reset();
    selectedBlocks.clear();
    highlightedConns.clear();
    routeSamplesMs.clear();
    dragKind = DragKind::None;

    Topology top = generate(blockCount);
    currentBlockCount = static_cast<unsigned>(top.blocks.size());
    currentConnCount = static_cast<unsigned>(top.conns.size());

    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
    for (const auto &b : top.blocks) {
        minX = std::min(minX, b.rect.l);
        minY = std::min(minY, b.rect.t);
        maxX = std::max(maxX, b.rect.r);
        maxY = std::max(maxY, b.rect.b);
    }
    worldBounds = {minX, minY, maxX, maxY};

    localroute::Config cfg;
    cfg.rerouteThirdPartyBlocked = thirdPartyReroute;
    localroute::LocalOrthogonalRouter initial(cfg);
    for (auto &b : top.blocks)
        initial.addBlock(b);
    for (const auto &c : top.conns)
        initial.addConnector(c);

    QElapsedTimer timer;
    timer.start();
    if (mode == RouteMode::Sync) {
        syncRouter = std::make_unique<localroute::LocalOrthogonalRouter>(std::move(initial));
        syncRouter->routeAllParallel();
    } else {
        asyncRouter = std::make_unique<localroute::AsyncLocalRouter>(std::move(initial));
    }
    double setupMs = double(timer.nsecsElapsed()) / 1e6;

    rebuildRouteCache();
    resetLocalView();

    if (mode == RouteMode::Async && asyncRouter) {
        QElapsedTimer startTimer;
        startTimer.start();
        asyncRouter->prioritize(worldViewportRect(), {});
        asyncRouter->startAll();
        setupMs += double(startTimer.nsecsElapsed()) / 1e6;
        rebuildRouteCache();
        asyncTimer.start();
    }

    routeTimeMs = setupMs;
    recordRouteSample(setupMs);
    update();
    emit statusChanged();
}

void CanvasWidget::setRouteMode(RouteMode newMode) {
    if (mode == newMode)
        return;
    asyncTimer.stop();
    localroute::LocalOrthogonalRouter model = extractModelCopy();
    asyncRouter.reset();
    syncRouter.reset();
    mode = newMode;

    QElapsedTimer timer;
    timer.start();
    if (mode == RouteMode::Sync) {
        syncRouter = std::make_unique<localroute::LocalOrthogonalRouter>(std::move(model));
        syncRouter->routeAllParallel();
    } else {
        asyncRouter = std::make_unique<localroute::AsyncLocalRouter>(std::move(model));
        asyncRouter->prioritize(worldViewportRect(),
                                std::vector<localroute::Id>(selectedBlocks.begin(), selectedBlocks.end()));
        asyncRouter->startAll();
        asyncTimer.start();
    }
    routeTimeMs = double(timer.nsecsElapsed()) / 1e6;
    recordRouteSample(routeTimeMs);
    rebuildRouteCache();
    update();
    emit statusChanged();
}

void CanvasWidget::setRerouteThirdPartyBlocked(bool enabled) {
    if (thirdPartyReroute == enabled)
        return;
    thirdPartyReroute = enabled;
    if (mode == RouteMode::Sync && syncRouter)
        syncRouter->setRerouteThirdPartyBlocked(enabled);
    else if (mode == RouteMode::Async && asyncRouter)
        asyncRouter->setRerouteThirdPartyBlocked(enabled);
    update();
    emit statusChanged();
}

void CanvasWidget::routeAll() {
    QElapsedTimer timer;
    timer.start();
    if (mode == RouteMode::Sync && syncRouter) {
        syncRouter->routeAllParallel();
        routeTimeMs = double(timer.nsecsElapsed()) / 1e6;
    } else if (mode == RouteMode::Async && asyncRouter) {
        asyncRouter->prioritize(worldViewportRect(),
                                std::vector<localroute::Id>(selectedBlocks.begin(), selectedBlocks.end()));
        asyncRouter->startAll();
        routeTimeMs = double(timer.nsecsElapsed()) / 1e6;
        asyncTimer.start();
    }
    recordRouteSample(routeTimeMs);
    rebuildRouteCache();
    update();
    emit statusChanged();
}

void CanvasWidget::fitAll() {
    double bw = std::max(1.0, worldBounds.r - worldBounds.l);
    double bh = std::max(1.0, worldBounds.b - worldBounds.t);
    double vw = std::max(100.0, double(width()) - 40.0);
    double vh = std::max(100.0, double(height()) - 40.0);
    zoom = std::clamp(std::min(vw / bw, vh / bh), 0.05, 4.0);
    double cx = (worldBounds.l + worldBounds.r) * 0.5;
    double cy = (worldBounds.t + worldBounds.b) * 0.5;
    panOffset = QPointF(width() * 0.5 - cx * zoom, height() * 0.5 - cy * zoom);
    updateViewportPriority();
    update();
}

void CanvasWidget::resetLocalView() {
    zoom = 1.0;
    const auto &view = activeView();
    const auto &blocks = view.allBlocks();
    if (!blocks.empty()) {
        localroute::Id midId = currentBlockCount / 2;
        auto it = blocks.find(midId);
        if (it == blocks.end())
            it = blocks.begin();
        const auto &r = it->second.rect;
        double cx = (r.l + r.r) * 0.5;
        double cy = (r.t + r.b) * 0.5;
        panOffset = QPointF(width() * 0.5 - cx * zoom, height() * 0.5 - cy * zoom);
    } else {
        panOffset = QPointF(40.0, 40.0);
    }
    updateViewportPriority();
    update();
}

void CanvasWidget::rebuildRouteCache() {
    const auto &view = activeView();
    const auto &conns = view.allConns();
    const auto &routes = view.allRoutes();
    const auto &blocks = view.allBlocks();

    routeCache.clear();
    routeCache.reserve(conns.size());
    routeCacheIndex.clear();
    routeCacheIndex.reserve(conns.size());
    statusCounts.fill(0);

    for (const auto &kv : conns) {
        const auto &c = kv.second;
        CachedRoute entry;
        entry.id = c.id;
        auto rit = routes.find(c.id);
        if (rit == routes.end() || rit->second.status == localroute::Status::Pending || rit->second.points.empty()) {
            entry.status = localroute::Status::Pending;
            const auto &sb = blocks.at(c.src);
            const auto &db = blocks.at(c.dst);
            entry.bounds = localroute::box(pinPos(sb, c.sp), pinPos(db, c.dp));
        } else {
            entry.status = rit->second.status;
            entry.bounds = pointsBounds(rit->second.points);
        }
        ++statusCounts[size_t(entry.status)];
        routeCacheIndex.emplace(c.id, routeCache.size());
        routeCache.push_back(entry);
    }
}

void CanvasWidget::updateRouteCacheForIds(const std::vector<localroute::Id> &ids) {
    const auto &view = activeView();
    const auto &routes = view.allRoutes();
    const auto &conns = view.allConns();
    const auto &blocks = view.allBlocks();

    for (localroute::Id id : ids) {
        auto idxIt = routeCacheIndex.find(id);
        if (idxIt == routeCacheIndex.end())
            continue;
        CachedRoute &entry = routeCache[idxIt->second];
        --statusCounts[size_t(entry.status)];

        auto rit = routes.find(id);
        if (rit == routes.end() || rit->second.status == localroute::Status::Pending || rit->second.points.empty()) {
            entry.status = localroute::Status::Pending;
            const auto &c = conns.at(id);
            entry.bounds = localroute::box(pinPos(blocks.at(c.src), c.sp), pinPos(blocks.at(c.dst), c.dp));
        } else {
            entry.status = rit->second.status;
            entry.bounds = pointsBounds(rit->second.points);
        }
        ++statusCounts[size_t(entry.status)];
    }
}

void CanvasWidget::updateRouteBoundsFromBlock(localroute::Id blockId) {
    const auto &view = activeView();
    const auto &conns = view.allConns();
    const auto &blocks = view.allBlocks();
    for (localroute::Id cid : view.adjacentConns(blockId)) {
        auto idxIt = routeCacheIndex.find(cid);
        if (idxIt == routeCacheIndex.end())
            continue;
        CachedRoute &entry = routeCache[idxIt->second];
        --statusCounts[size_t(entry.status)];
        const auto &c = conns.at(cid);
        const auto &routes = view.allRoutes();
        auto rit = routes.find(cid);
        entry.status = (rit == routes.end()) ? localroute::Status::Pending : rit->second.status;
        if (entry.status == localroute::Status::Pending || rit->second.points.empty()) {
            entry.bounds = localroute::box(pinPos(blocks.at(c.src), c.sp), pinPos(blocks.at(c.dst), c.dp));
        } else {
            entry.bounds = pointsBounds(rit->second.points);
        }
        ++statusCounts[size_t(entry.status)];
    }
}

void CanvasWidget::recordRouteSample(double ms) {
    routeSamplesMs.push_back(ms);
    if (routeSamplesMs.size() > 100)
        routeSamplesMs.pop_front();
}

double CanvasWidget::routeP99Ms() const {
    if (routeSamplesMs.empty())
        return 0.0;
    std::vector<double> tmp(routeSamplesMs.begin(), routeSamplesMs.end());
    std::sort(tmp.begin(), tmp.end());
    size_t idx = size_t(std::ceil(0.99 * tmp.size())) - 1;
    return tmp[std::min(idx, tmp.size() - 1)];
}

void CanvasWidget::updateViewportPriority() {
    if (mode == RouteMode::Async && asyncRouter) {
        asyncRouter->prioritize(worldViewportRect(),
                                std::vector<localroute::Id>(selectedBlocks.begin(), selectedBlocks.end()));
    }
}

localroute::Rect CanvasWidget::worldViewportRect() const {
    QPointF tl = screenToWorld(QPointF(0, 0));
    QPointF br = screenToWorld(QPointF(width(), height()));
    return {std::min(tl.x(), br.x()), std::min(tl.y(), br.y()),
            std::max(tl.x(), br.x()), std::max(tl.y(), br.y())};
}

QPointF CanvasWidget::screenToWorld(const QPointF &p) const {
    return {(p.x() - panOffset.x()) / zoom, (p.y() - panOffset.y()) / zoom};
}

QPointF CanvasWidget::worldToScreen(const QPointF &p) const {
    return {p.x() * zoom + panOffset.x(), p.y() * zoom + panOffset.y()};
}

localroute::Id CanvasWidget::hitTestBlock(const QPointF &worldPos) const {
    const auto &view = activeView();
    localroute::Rect probe{worldPos.x() - 1.0, worldPos.y() - 1.0, worldPos.x() + 1.0, worldPos.y() + 1.0};
    auto ids = view.queryBlocks(probe);
    for (auto it = ids.rbegin(); it != ids.rend(); ++it) {
        const auto &r = view.block(*it).rect;
        if (worldPos.x() >= r.l && worldPos.x() <= r.r && worldPos.y() >= r.t && worldPos.y() <= r.b)
            return *it;
    }
    return std::numeric_limits<localroute::Id>::max();
}

void CanvasWidget::stepSmokeDrag(const std::vector<localroute::Id> &ids, double dx, double dy) {
    if (ids.empty())
        return;
    const auto &view = activeView();
    std::vector<std::pair<localroute::Id, localroute::Rect>> moves;
    moves.reserve(ids.size());
    for (localroute::Id id : ids) {
        auto r = view.block(id).rect;
        moves.push_back({id, {r.l + dx, r.t + dy, r.r + dx, r.b + dy}});
    }
    QElapsedTimer timer;
    timer.start();
    if (mode == RouteMode::Sync && syncRouter) {
        syncRouter->moveBlocksBatch(moves);
        auto changed = syncRouter->rerouteAffected(ids);
        routeTimeMs = double(timer.nsecsElapsed()) / 1e6;
        updateRouteCacheForIds(changed);
    } else if (mode == RouteMode::Async && asyncRouter) {
        asyncRouter->move(moves);
        std::vector<localroute::Id> updated;
        asyncRouter->poll(128, &updated);
        routeTimeMs = double(timer.nsecsElapsed()) / 1e6;
        for (localroute::Id id : ids)
            updateRouteBoundsFromBlock(id);
        if (!updated.empty())
            updateRouteCacheForIds(updated);
    }
    recordRouteSample(routeTimeMs);
}

void CanvasWidget::onAsyncTick() {
    if (mode != RouteMode::Async || !asyncRouter)
        return;
    const size_t batchSize = (dragKind == DragKind::MoveBlocks || asyncRouter->pendingCount() <= 256) ? 128 : 512;
    std::vector<localroute::Id> updated;
    QElapsedTimer timer;
    timer.start();
    size_t n = asyncRouter->poll(batchSize, &updated);
    double pollMs = double(timer.nsecsElapsed()) / 1e6;
    if (n > 0) {
        if (dragKind != DragKind::MoveBlocks) {
            routeTimeMs = pollMs;
            recordRouteSample(pollMs);
        }
        updateRouteCacheForIds(updated);
        update();
        emit statusChanged();
    } else if (asyncRouter->pendingCount() == 0 && dragKind != DragKind::MoveBlocks) {
        asyncTimer.stop();
        emit statusChanged();
    }
}

void CanvasWidget::wheelEvent(QWheelEvent *event) {
    QPointF cursor = event->pos();
    QPointF worldBefore = screenToWorld(cursor);
    double steps = double(event->angleDelta().y()) / 120.0;
    if (steps == 0.0)
        return;
    double factor = std::pow(1.15, steps);
    double newZoom = std::clamp(zoom * factor, 0.05, 4.0);
    zoom = newZoom;
    panOffset = QPointF(cursor.x() - worldBefore.x() * zoom, cursor.y() - worldBefore.y() * zoom);
    updateViewportPriority();
    update();
}

void CanvasWidget::mousePressEvent(QMouseEvent *event) {
    lastMousePos = event->pos();
    if (event->button() == Qt::MiddleButton || (event->button() == Qt::LeftButton && spacePressed)) {
        dragKind = DragKind::Pan;
        setCursor(Qt::ClosedHandCursor);
        return;
    }
    if (event->button() != Qt::LeftButton)
        return;

    QPointF worldPos = screenToWorld(event->pos());
    localroute::Id hit = hitTestBlock(worldPos);
    const auto &view = activeView();

    if (hit != std::numeric_limits<localroute::Id>::max()) {
        if (event->modifiers() & Qt::ControlModifier) {
            if (selectedBlocks.count(hit))
                selectedBlocks.erase(hit);
            else
                selectedBlocks.insert(hit);
        } else if (!selectedBlocks.count(hit)) {
            selectedBlocks.clear();
            selectedBlocks.insert(hit);
        }
        dragKind = DragKind::MoveBlocks;
        dragStartWorld = worldPos;
        dragInitialRects.clear();
        highlightedConns.clear();
        for (localroute::Id id : selectedBlocks) {
            dragInitialRects.emplace(id, view.block(id).rect);
            for (localroute::Id cid : view.adjacentConns(id))
                highlightedConns.insert(cid);
        }
        updateViewportPriority();
        update();
    } else {
        if (!(event->modifiers() & Qt::ControlModifier))
            selectedBlocks.clear();
        highlightedConns.clear();
        dragKind = DragKind::RubberBand;
        dragStartWorld = worldPos;
        dragCurrentWorld = worldPos;
        update();
    }
}

void CanvasWidget::mouseMoveEvent(QMouseEvent *event) {
    QPointF pos = event->pos();
    if (dragKind == DragKind::Pan) {
        QPointF delta = pos - lastMousePos;
        lastMousePos = pos;
        panOffset += delta;
        updateViewportPriority();
        update();
        return;
    }
    lastMousePos = pos;

    if (dragKind == DragKind::RubberBand) {
        dragCurrentWorld = screenToWorld(pos);
        update();
        return;
    }

    if (dragKind == DragKind::MoveBlocks && !dragInitialRects.empty()) {
        QPointF worldPos = screenToWorld(pos);
        double dx = worldPos.x() - dragStartWorld.x();
        double dy = worldPos.y() - dragStartWorld.y();
        std::vector<std::pair<localroute::Id, localroute::Rect>> moves;
        std::vector<localroute::Id> movedIds;
        moves.reserve(dragInitialRects.size());
        movedIds.reserve(dragInitialRects.size());
        for (const auto &kv : dragInitialRects) {
            const auto &r = kv.second;
            moves.push_back({kv.first, {r.l + dx, r.t + dy, r.r + dx, r.b + dy}});
            movedIds.push_back(kv.first);
        }

        QElapsedTimer timer;
        timer.start();
        if (mode == RouteMode::Sync && syncRouter) {
            syncRouter->moveBlocksBatch(moves);
            auto changed = syncRouter->rerouteAffected(movedIds);
            routeTimeMs = double(timer.nsecsElapsed()) / 1e6;
            updateRouteCacheForIds(changed);
            if (thirdPartyReroute) {
                highlightedConns.clear();
                for (localroute::Id cid : changed)
                    highlightedConns.insert(cid);
            }
        } else if (mode == RouteMode::Async && asyncRouter) {
            asyncRouter->move(moves);
            std::vector<localroute::Id> updated;
            asyncRouter->poll(128, &updated);
            routeTimeMs = double(timer.nsecsElapsed()) / 1e6;
            for (localroute::Id id : movedIds)
                updateRouteBoundsFromBlock(id);
            if (!updated.empty())
                updateRouteCacheForIds(updated);
            if (!asyncTimer.isActive())
                asyncTimer.start();
        }
        recordRouteSample(routeTimeMs);
        update();
        emit statusChanged();
    }
}

void CanvasWidget::mouseReleaseEvent(QMouseEvent *event) {
    if (dragKind == DragKind::Pan) {
        dragKind = DragKind::None;
        setCursor(spacePressed ? Qt::OpenHandCursor : Qt::ArrowCursor);
        return;
    }
    if (event->button() != Qt::LeftButton)
        return;

    if (dragKind == DragKind::RubberBand) {
        localroute::Rect selBox{std::min(dragStartWorld.x(), dragCurrentWorld.x()),
                                std::min(dragStartWorld.y(), dragCurrentWorld.y()),
                                std::max(dragStartWorld.x(), dragCurrentWorld.x()),
                                std::max(dragStartWorld.y(), dragCurrentWorld.y())};
        const auto &view = activeView();
        for (localroute::Id id : view.queryBlocks(selBox)) {
            if (view.block(id).rect.intersects(selBox))
                selectedBlocks.insert(id);
        }
        highlightedConns.clear();
        for (localroute::Id id : selectedBlocks) {
            for (localroute::Id cid : view.adjacentConns(id))
                highlightedConns.insert(cid);
        }
        dragKind = DragKind::None;
        updateViewportPriority();
        update();
        return;
    }

    if (dragKind == DragKind::MoveBlocks) {
        dragKind = DragKind::None;
        if (mode == RouteMode::Sync && syncRouter) {
            std::vector<localroute::Id> movedIds;
            for (const auto &kv : dragInitialRects)
                movedIds.push_back(kv.first);
            QElapsedTimer timer;
            timer.start();
            auto changed = syncRouter->rerouteAffected(movedIds, true);
            routeTimeMs = double(timer.nsecsElapsed()) / 1e6;
            recordRouteSample(routeTimeMs);
            updateRouteCacheForIds(changed);
        } else if (mode == RouteMode::Async && asyncRouter) {
            asyncRouter->settle();
            if (!asyncTimer.isActive())
                asyncTimer.start();
        }
        dragInitialRects.clear();
        update();
        emit statusChanged();
    }
}

void CanvasWidget::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_Space && !event->isAutoRepeat()) {
        spacePressed = true;
        if (dragKind == DragKind::None)
            setCursor(Qt::OpenHandCursor);
    } else if (event->key() == Qt::Key_R) {
        resetLocalView();
    } else if (event->key() == Qt::Key_F) {
        fitAll();
    } else {
        QWidget::keyPressEvent(event);
    }
}

void CanvasWidget::keyReleaseEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_Space && !event->isAutoRepeat()) {
        spacePressed = false;
        if (dragKind == DragKind::None)
            setCursor(Qt::ArrowCursor);
    } else {
        QWidget::keyReleaseEvent(event);
    }
}

void CanvasWidget::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    updateViewportPriority();
}

void CanvasWidget::paintEvent(QPaintEvent *) {
    QElapsedTimer paintTimer;
    paintTimer.start();

    qint64 nowMs = fpsClock.elapsed();
    frameTimestampsMs.push_back(nowMs);
    while (frameTimestampsMs.size() > 1 && nowMs - frameTimestampsMs.front() > 1000)
        frameTimestampsMs.pop_front();
    if (frameTimestampsMs.size() >= 2) {
        qint64 spanMs = nowMs - frameTimestampsMs.front();
        currentFps = spanMs > 0 ? double(frameTimestampsMs.size() - 1) * 1000.0 / double(spanMs) : 0.0;
    }

    QPainter painter(this);
    painter.fillRect(rect(), QColor(246, 248, 251));

    const bool highDetail = zoom >= 0.4;
    const bool midDetail = zoom >= 0.15 && zoom < 0.4;
    const bool lowDetail = zoom < 0.15;

    painter.save();
    painter.translate(panOffset);
    painter.scale(zoom, zoom);

    const localroute::Rect vp = worldViewportRect();
    const auto &view = activeView();
    const auto &routes = view.allRoutes();

    // 1. Collect visible wires via O(1) cached routeBounds culling + segment viewport clamping.
    painter.setRenderHint(QPainter::Antialiasing, false);
    QPen validPen(QColor(30, 78, 140), lowDetail ? 1.0 : 1.35);
    validPen.setCosmetic(true);
    QPen pendingPen(lowDetail ? QColor(115, 134, 158, 140) : QColor(115, 134, 158), 1.2,
                    lowDetail ? Qt::SolidLine : Qt::DashLine);
    pendingPen.setCosmetic(true);
    QPen displayPen(QColor(230, 126, 34), 1.5);
    displayPen.setCosmetic(true);
    QPen hiValidPen(QColor(0, 120, 215), 2.2);
    hiValidPen.setCosmetic(true);
    QPen hiPendingPen(QColor(0, 120, 215), 1.8, Qt::DashLine);
    hiPendingPen.setCosmetic(true);

    QVector<QLineF> validSegs, pendingSegs, displaySegs;
    validSegs.reserve(2048);
    pendingSegs.reserve(1024);
    displaySegs.reserve(128);
    std::vector<localroute::Id> topHighlight;
    topHighlight.reserve(highlightedConns.size());

    auto appendClippedSegs = [&](QVector<QLineF> &dst, localroute::Point p0, localroute::Point p1) {
        if (p0.x == p1.x) {
            if (p0.x < vp.l || p0.x > vp.r)
                return;
            double y0 = std::clamp(std::min(p0.y, p1.y), vp.t, vp.b);
            double y1 = std::clamp(std::max(p0.y, p1.y), vp.t, vp.b);
            if (y0 < y1)
                dst.append(QLineF(p0.x, y0, p0.x, y1));
        } else if (p0.y == p1.y) {
            if (p0.y < vp.t || p0.y > vp.b)
                return;
            double x0 = std::clamp(std::min(p0.x, p1.x), vp.l, vp.r);
            double x1 = std::clamp(std::max(p0.x, p1.x), vp.l, vp.r);
            if (x0 < x1)
                dst.append(QLineF(x0, p0.y, x1, p0.y));
        } else {
            if (localroute::box(p0, p1).intersects(vp))
                dst.append(QLineF(p0.x, p0.y, p1.x, p1.y));
        }
    };

    const auto &allBlocks = view.allBlocks();
    const auto &allConns = view.allConns();
    size_t drawnConns = 0;
    for (const auto &entry : routeCache) {
        if (highlightedConns.count(entry.id)) {
            topHighlight.push_back(entry.id);
            continue;
        }
        if (!entry.bounds.intersects(vp))
            continue;
        ++drawnConns;
        if (entry.status == localroute::Status::Pending) {
            const auto &c = allConns.at(entry.id);
            localroute::Point a = pinPos(allBlocks.at(c.src), c.sp);
            localroute::Point b = pinPos(allBlocks.at(c.dst), c.dp);
            if (a.x == b.x || a.y == b.y) {
                appendClippedSegs(pendingSegs, a, b);
            } else {
                localroute::Point corner{b.x, a.y};
                appendClippedSegs(pendingSegs, a, corner);
                appendClippedSegs(pendingSegs, corner, b);
            }
        } else {
            auto rit = routes.find(entry.id);
            if (rit == routes.end() || rit->second.points.size() < 2)
                continue;
            auto &dst = (entry.status == localroute::Status::Valid) ? validSegs : displaySegs;
            const auto &pts = rit->second.points;
            for (size_t j = 1; j < pts.size(); ++j)
                appendClippedSegs(dst, pts[j - 1], pts[j]);
        }
    }

    if (!validSegs.isEmpty()) {
        painter.setPen(validPen);
        painter.drawLines(validSegs);
    }
    if (!pendingSegs.isEmpty()) {
        painter.setPen(pendingPen);
        painter.drawLines(pendingSegs);
    }
    if (!displaySegs.isEmpty()) {
        painter.setPen(displayPen);
        painter.drawLines(displaySegs);
    }

    // 2. Draw visible blocks via SpatialGridIndex viewport query + LOD.
    auto visIds = view.queryBlocks(vp);
    visibleBlockCount = visIds.size();

    if (highDetail)
        painter.setRenderHint(QPainter::Antialiasing, true);

    QPen blockPen(QColor(72, 86, 106), 1.2);
    blockPen.setCosmetic(true);
    QPen selBlockPen(QColor(0, 120, 215), 2.2);
    selBlockPen.setCosmetic(true);
    QBrush blockBrush(QColor(255, 255, 255));
    QBrush selBlockBrush(QColor(227, 241, 255));
    QBrush lowBlockBrush(QColor(186, 200, 218));
    QBrush pinBrush(QColor(46, 64, 87));

    QFont labelFont = painter.font();
    labelFont.setPixelSize(11);
    painter.setFont(labelFont);

    if (lowDetail) {
        QVector<QRectF> normalRects, selRects;
        normalRects.reserve(visIds.size());
        for (localroute::Id id : visIds) {
            const auto &b = view.block(id);
            QRectF r(b.rect.l, b.rect.t, b.rect.r - b.rect.l, b.rect.b - b.rect.t);
            if (selectedBlocks.count(id))
                selRects.append(r);
            else
                normalRects.append(r);
        }
        painter.setPen(Qt::NoPen);
        if (!normalRects.isEmpty()) {
            painter.setBrush(lowBlockBrush);
            painter.drawRects(normalRects);
        }
        if (!selRects.isEmpty()) {
            painter.setBrush(QColor(0, 120, 215));
            painter.drawRects(selRects);
        }
    } else {
        for (localroute::Id id : visIds) {
            const auto &b = view.block(id);
            const bool selected = selectedBlocks.count(id) != 0;
            QRectF r(b.rect.l, b.rect.t, b.rect.r - b.rect.l, b.rect.b - b.rect.t);
            if (midDetail) {
                painter.setPen(selected ? selBlockPen : blockPen);
                painter.setBrush(selected ? selBlockBrush : blockBrush);
                painter.drawRect(r);
            } else {
                painter.setPen(selected ? selBlockPen : blockPen);
                painter.setBrush(selected ? selBlockBrush : blockBrush);
                painter.drawRoundedRect(r, 4.0, 4.0);
                painter.setPen(QColor(38, 50, 66));
                painter.drawText(r, Qt::AlignCenter, QString("B%1").arg(id));
                painter.setPen(Qt::NoPen);
                painter.setBrush(pinBrush);
                for (const auto &p : b.pins) {
                    auto pt = pinPos(b, p.id);
                    painter.drawEllipse(QPointF(pt.x, pt.y), 2.8, 2.8);
                }
            }
        }
    }

    // 3. Draw dragged/selected highlighted wires on top.
    painter.setRenderHint(QPainter::Antialiasing, highDetail);
    QVector<QLineF> hiValidSegs, hiPendingSegs, hiDisplaySegs;
    for (localroute::Id cid : topHighlight) {
        ++drawnConns;
        auto rit = routes.find(cid);
        if (rit == routes.end() || rit->second.status == localroute::Status::Pending || rit->second.points.empty()) {
            const auto &c = allConns.at(cid);
            localroute::Point a = pinPos(allBlocks.at(c.src), c.sp);
            localroute::Point b = pinPos(allBlocks.at(c.dst), c.dp);
            if (a.x == b.x || a.y == b.y) {
                appendClippedSegs(hiPendingSegs, a, b);
            } else {
                localroute::Point corner{b.x, a.y};
                appendClippedSegs(hiPendingSegs, a, corner);
                appendClippedSegs(hiPendingSegs, corner, b);
            }
        } else {
            auto &dst = (rit->second.status == localroute::Status::Valid) ? hiValidSegs : hiDisplaySegs;
            const auto &pts = rit->second.points;
            for (size_t j = 1; j < pts.size(); ++j)
                appendClippedSegs(dst, pts[j - 1], pts[j]);
        }
    }
    if (!hiValidSegs.isEmpty()) {
        painter.setPen(hiValidPen);
        painter.drawLines(hiValidSegs);
    }
    if (!hiPendingSegs.isEmpty()) {
        painter.setPen(hiPendingPen);
        painter.drawLines(hiPendingSegs);
    }
    if (!hiDisplaySegs.isEmpty()) {
        painter.setPen(displayPen);
        painter.drawLines(hiDisplaySegs);
    }
    visibleConnCount = drawnConns;

    // 4. Rubber-band box
    if (dragKind == DragKind::RubberBand) {
        QRectF rb(std::min(dragStartWorld.x(), dragCurrentWorld.x()),
                  std::min(dragStartWorld.y(), dragCurrentWorld.y()),
                  std::abs(dragCurrentWorld.x() - dragStartWorld.x()),
                  std::abs(dragCurrentWorld.y() - dragStartWorld.y()));
        QPen rbPen(QColor(0, 120, 215), 1.4, Qt::DashLine);
        rbPen.setCosmetic(true);
        painter.setPen(rbPen);
        painter.setBrush(QColor(0, 120, 215, 36));
        painter.drawRect(rb);
    }

    painter.restore();

    paintTimeMs = double(paintTimer.nsecsElapsed()) / 1e6;
    drawHud(painter);
}

void CanvasWidget::drawHud(QPainter &painter) {
    painter.save();
    painter.setRenderHint(QPainter::Antialiasing, true);

    QRectF hudRect(12, 12, 370, 152);
    painter.setPen(QPen(QColor(45, 58, 76, 180), 1.0));
    painter.setBrush(QColor(18, 24, 34, 210));
    painter.drawRoundedRect(hudRect, 8.0, 8.0);

    QFont f("Consolas", 9);
    if (!f.exactMatch())
        f = QFont("Monospace", 9);
    painter.setFont(f);
    painter.setPen(QColor(235, 242, 250));

    const char *modeStr = (mode == RouteMode::Async) ? "Async" : "Sync";
    const char *rerouteStr = thirdPartyReroute ? "ON" : "OFF";
    QString lines[6] = {
        QString("Scale       : %1 blocks / %2 conns").arg(currentBlockCount).arg(currentConnCount),
        QString("Mode        : %1 | ThirdPartyReroute: %2").arg(modeStr).arg(rerouteStr),
        QString("FPS / Zoom  : %1 fps | %2x").arg(currentFps, 0, 'f', 1).arg(zoom, 0, 'f', 2),
        QString("Route Time  : %1 ms (P99: %2 ms)").arg(routeTimeMs, 0, 'f', 3).arg(routeP99Ms(), 0, 'f', 3),
        QString("Paint Time  : %1 ms (Vis: %2 blks / %3 conns)")
            .arg(paintTimeMs, 0, 'f', 3)
            .arg(visibleBlockCount)
            .arg(visibleConnCount),
        QString("Route Status: Valid %1 | Pending %2 | Display %3")
            .arg(statusCounts[size_t(localroute::Status::Valid)])
            .arg(statusCounts[size_t(localroute::Status::Pending)])
            .arg(statusCounts[size_t(localroute::Status::DisplayOnly)] +
                 statusCounts[size_t(localroute::Status::Failed)])};

    int y = 34;
    for (const QString &line : lines) {
        painter.drawText(24, y, line);
        y += 22;
    }
    painter.restore();
}
