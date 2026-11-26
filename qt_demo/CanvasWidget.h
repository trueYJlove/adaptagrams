#pragma once

#include "localroute/AsyncLocalRouter.h"
#include "localroute/LocalOrthogonalRouter.h"
#include <QElapsedTimer>
#include <QPointF>
#include <QRectF>
#include <QTimer>
#include <QWidget>
#include <array>
#include <deque>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class CanvasWidget : public QWidget {
    Q_OBJECT
  public:
    enum class RouteMode { Async, Sync };

    explicit CanvasWidget(QWidget *parent = nullptr);
    ~CanvasWidget() override;

    void loadTopology(unsigned blockCount);
    void setRouteMode(RouteMode mode);
    void setRerouteThirdPartyBlocked(bool enabled);
    void routeAll();
    void fitAll();
    void resetLocalView();

    unsigned blockCount() const { return currentBlockCount; }
    unsigned connCount() const { return currentConnCount; }
    RouteMode routeMode() const { return mode; }
    bool rerouteThirdPartyBlocked() const { return thirdPartyReroute; }

    // Programmatic smoke-test helpers for automated verification.
    double lastRouteMs() const { return routeTimeMs; }
    double routeP99Ms() const;
    double lastPaintMs() const { return paintTimeMs; }
    size_t lastVisibleBlocks() const { return visibleBlockCount; }
    size_t lastVisibleConns() const { return visibleConnCount; }
    void stepSmokeDrag(const std::vector<localroute::Id> &ids, double dx, double dy);

  signals:
    void statusChanged();

  protected:
    void paintEvent(QPaintEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

  private slots:
    void onAsyncTick();

  private:
    struct CachedRoute {
        localroute::Id id = 0;
        localroute::Rect bounds{0, 0, 0, 0};
        localroute::Status status = localroute::Status::Pending;
    };

    const localroute::LocalOrthogonalRouter &activeView() const;
    localroute::LocalOrthogonalRouter extractModelCopy() const;
    void rebuildRouteCache();
    void updateRouteCacheForIds(const std::vector<localroute::Id> &ids);
    void updateRouteBoundsFromBlock(localroute::Id blockId);
    void recordRouteSample(double ms);
    void updateViewportPriority();
    localroute::Rect worldViewportRect() const;
    QPointF screenToWorld(const QPointF &p) const;
    QPointF worldToScreen(const QPointF &p) const;
    localroute::Id hitTestBlock(const QPointF &worldPos) const;
    void drawHud(QPainter &painter);

    RouteMode mode = RouteMode::Async;
    bool thirdPartyReroute = false;
    unsigned currentBlockCount = 0;
    unsigned currentConnCount = 0;
    localroute::Rect worldBounds{0, 0, 1000, 1000};

    std::unique_ptr<localroute::LocalOrthogonalRouter> syncRouter;
    std::unique_ptr<localroute::AsyncLocalRouter> asyncRouter;

    std::vector<CachedRoute> routeCache;
    std::unordered_map<localroute::Id, size_t> routeCacheIndex;
    std::array<size_t, 4> statusCounts{};

    // Camera transform: screen = world * zoom + panOffset
    double zoom = 1.0;
    QPointF panOffset{40.0, 40.0};

    // Interaction state
    enum class DragKind { None, Pan, MoveBlocks, RubberBand };
    DragKind dragKind = DragKind::None;
    bool spacePressed = false;
    QPointF lastMousePos;
    QPointF dragStartWorld;
    QPointF dragCurrentWorld;
    std::unordered_set<localroute::Id> selectedBlocks;
    std::unordered_map<localroute::Id, localroute::Rect> dragInitialRects;
    std::unordered_set<localroute::Id> highlightedConns;

    // Async polling & telemetry
    QTimer asyncTimer;
    QElapsedTimer fpsClock;
    std::deque<qint64> frameTimestampsMs;
    std::deque<double> routeSamplesMs;
    double routeTimeMs = 0.0;
    double paintTimeMs = 0.0;
    double currentFps = 0.0;
    size_t visibleBlockCount = 0;
    size_t visibleConnCount = 0;
};
