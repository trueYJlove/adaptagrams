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
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "library_widget.h"
#include "localroute/async_local_router.h"
#include "localroute/local_orthogonal_router.h"

class CanvasWidget : public QWidget {
  Q_OBJECT

 public:
  enum class RouteMode { kAsync, kSync };

  explicit CanvasWidget(QWidget* parent = nullptr);
  ~CanvasWidget() override;

  void LoadTopology(unsigned block_count);
  void LoadPresetParallelBundle();
  void LoadPresetCascade();
  void SetRouteMode(RouteMode mode);
  void SetRerouteThirdPartyBlocked(bool enabled);
  void SetEnableChannelNudging(bool enabled);
  void RouteAll();
  void FitAll();
  void ResetLocalView();

  localroute::Id AddBlockFromTemplate(const BlockTemplate& tmpl,
                                      QPointF world_center);
  localroute::Id SpawnBlockAtCenter(const BlockTemplate& tmpl);
  bool GetSelectedBlockPorts(localroute::Id* block_id, int* left, int* right,
                             int* top, int* bottom) const;
  void SetSelectedBlockPorts(int left, int right, int top, int bottom);
  void SetBlockPorts(localroute::Id block_id, int left, int right, int top,
                     int bottom);
  localroute::Id AddWire(localroute::Id src_block, localroute::Id src_pin,
                         localroute::Id dst_block, localroute::Id dst_pin);
  void DeleteSelection();
  void ClearBlockWires(localroute::Id block_id);

  unsigned block_count() const { return current_block_count_; }
  unsigned conn_count() const { return current_conn_count_; }
  RouteMode route_mode() const { return mode_; }
  bool reroute_third_party_blocked() const { return third_party_reroute_; }
  bool enable_channel_nudging() const { return channel_nudging_; }
  std::size_t selected_block_count() const { return selected_blocks_.size(); }
  bool has_selected_conn() const { return has_selected_conn_; }
  const localroute::LocalOrthogonalRouter& router_view() const {
    return ActiveView();
  }

  double last_route_ms() const { return route_time_ms_; }
  double RouteP99Ms() const;
  double last_paint_ms() const { return paint_time_ms_; }
  std::size_t last_visible_blocks() const { return visible_block_count_; }
  std::size_t last_visible_conns() const { return visible_conn_count_; }
  void StepSmokeDrag(const std::vector<localroute::Id>& ids, double dx,
                     double dy);

 signals:
  void StatusChanged();
  void SelectionChanged();

 protected:
  void paintEvent(QPaintEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dragMoveEvent(QDragMoveEvent* event) override;
  void dragLeaveEvent(QDragLeaveEvent* event) override;
  void dropEvent(QDropEvent* event) override;
  void contextMenuEvent(QContextMenuEvent* event) override;

 private slots:
  void OnAsyncTick();

 private:
  struct CachedRoute {
    localroute::Id id = 0;
    localroute::Rect bounds{0, 0, 0, 0};
    localroute::Status status = localroute::Status::kPending;
  };

  struct PinHit {
    bool valid = false;
    localroute::Id block_id = 0;
    localroute::Id pin_id = 0;
    localroute::Point world_pos{0.0, 0.0};
  };

  enum class DragKind { kNone, kPan, kMoveBlocks, kRubberBand, kWiring };

  const localroute::LocalOrthogonalRouter& ActiveView() const;
  void InstallInitialRouter(localroute::LocalOrthogonalRouter initial);
  void ApplyModelEdit(const std::function<std::vector<localroute::Id>(
                          localroute::LocalOrthogonalRouter*)>& edit_fn);
  void RefreshWorldBounds();
  void RebuildRouteCache();
  void UpdateRouteCacheForIds(const std::vector<localroute::Id>& ids);
  void UpdateRouteBoundsFromBlock(localroute::Id block_id);
  void RecordRouteSample(double ms);
  void UpdateViewportPriority();
  localroute::Rect WorldViewportRect() const;
  QPointF ScreenToWorld(const QPointF& p) const;
  QPointF WorldToScreen(const QPointF& p) const;
  localroute::Id HitTestBlock(const QPointF& world_pos) const;
  PinHit HitTestPin(const QPointF& screen_pos, double max_px = 12.0) const;
  bool HitTestWire(const QPointF& screen_pos,
                   localroute::Id* out_conn_id) const;
  void DrawHud(QPainter* painter);

  RouteMode mode_ = RouteMode::kAsync;
  bool third_party_reroute_ = false;
  bool channel_nudging_ = true;
  unsigned current_block_count_ = 0;
  unsigned current_conn_count_ = 0;
  localroute::Id next_block_id_ = 1;
  localroute::Id next_conn_id_ = 1;
  localroute::Rect world_bounds_{-400, -300, 400, 300};

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
  bool has_selected_conn_ = false;
  localroute::Id selected_conn_id_ = 0;

  // Interactive wiring & library drop ghost state
  PinHit hover_pin_;
  PinHit wire_src_;
  bool has_drop_ghost_ = false;
  BlockTemplate drop_ghost_tmpl_;
  QPointF drop_ghost_world_;

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
