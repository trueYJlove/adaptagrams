#ifndef LOCALROUTE_QT_DEMO_CANVAS_WIDGET_H_
#define LOCALROUTE_QT_DEMO_CANVAS_WIDGET_H_

#include <QElapsedTimer>
#include <QPointF>
#include <QRectF>
#include <QTimer>
#include <QWidget>
#include <array>
#include <cstddef>
#include <deque>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "localroute/async_local_router.h"
#include "localroute/local_orthogonal_router.h"

class CanvasWidget : public QWidget {
  Q_OBJECT

 public:
  enum class RouteMode { kAsync, kSync };

  explicit CanvasWidget(QWidget* parent = nullptr);
  ~CanvasWidget() override;

  void LoadTopology(unsigned block_count);
  void SetRouteMode(RouteMode mode);
  void SetRerouteThirdPartyBlocked(bool enabled);
  void RouteAll();
  void FitAll();
  void ResetLocalView();

  unsigned block_count() const { return current_block_count_; }
  unsigned conn_count() const { return current_conn_count_; }
  RouteMode route_mode() const { return mode_; }
  bool reroute_third_party_blocked() const { return third_party_reroute_; }

  double last_route_ms() const { return route_time_ms_; }
  double RouteP99Ms() const;
  double last_paint_ms() const { return paint_time_ms_; }
  std::size_t last_visible_blocks() const { return visible_block_count_; }
  std::size_t last_visible_conns() const { return visible_conn_count_; }
  void StepSmokeDrag(const std::vector<localroute::Id>& ids, double dx,
                     double dy);

 signals:
  void StatusChanged();

 protected:
  void paintEvent(QPaintEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private slots:
  void OnAsyncTick();

 private:
  struct CachedRoute {
    localroute::Id id = 0;
    localroute::Rect bounds{0, 0, 0, 0};
    localroute::Status status = localroute::Status::kPending;
  };

  enum class DragKind { kNone, kPan, kMoveBlocks, kRubberBand };

  const localroute::LocalOrthogonalRouter& ActiveView() const;
  localroute::LocalOrthogonalRouter ExtractModelCopy() const;
  void RebuildRouteCache();
  void UpdateRouteCacheForIds(const std::vector<localroute::Id>& ids);
  void UpdateRouteBoundsFromBlock(localroute::Id block_id);
  void RecordRouteSample(double ms);
  void UpdateViewportPriority();
  localroute::Rect WorldViewportRect() const;
  QPointF ScreenToWorld(const QPointF& p) const;
  localroute::Id HitTestBlock(const QPointF& world_pos) const;
  void DrawHud(QPainter* painter);

  RouteMode mode_ = RouteMode::kAsync;
  bool third_party_reroute_ = false;
  unsigned current_block_count_ = 0;
  unsigned current_conn_count_ = 0;
  localroute::Rect world_bounds_{0, 0, 1000, 1000};

  std::unique_ptr<localroute::LocalOrthogonalRouter> sync_router_;
  std::unique_ptr<localroute::AsyncLocalRouter> async_router_;

  std::vector<CachedRoute> route_cache_;
  std::unordered_map<localroute::Id, std::size_t> route_cache_index_;
  std::array<std::size_t, 4> status_counts_{};

  double zoom_ = 1.0;
  QPointF pan_offset_{40.0, 40.0};

  DragKind drag_kind_ = DragKind::kNone;
  bool space_pressed_ = false;
  QPointF last_mouse_pos_;
  QPointF drag_start_world_;
  QPointF drag_current_world_;
  std::unordered_set<localroute::Id> selected_blocks_;
  std::unordered_map<localroute::Id, localroute::Rect> drag_initial_rects_;
  std::unordered_set<localroute::Id> highlighted_conns_;

  QTimer async_timer_;
  QElapsedTimer fps_clock_;
  std::deque<qint64> frame_timestamps_ms_;
  std::deque<double> route_samples_ms_;
  double route_time_ms_ = 0.0;
  double paint_time_ms_ = 0.0;
  double current_fps_ = 0.0;
  std::size_t visible_block_count_ = 0;
  std::size_t visible_conn_count_ = 0;
};

#endif  // LOCALROUTE_QT_DEMO_CANVAS_WIDGET_H_
