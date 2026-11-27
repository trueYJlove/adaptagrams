#include "canvas_widget.h"

#include <QContextMenuEvent>
#include <QDragEnterEvent>
#include <QDragLeaveEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QKeyEvent>
#include <QLineF>
#include <QMenu>
#include <QMimeData>
#include <QMouseEvent>
#include <QPainter>
#include <QPolygonF>
#include <QVector>
#include <QWheelEvent>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

#include "topology_generator.h"

namespace {

localroute::Point PinPos(const localroute::Block& b, localroute::Id pin_id) {
  for (const auto& p : b.pins) {
    if (p.id == pin_id) {
      return {p.x == 1.0 ? b.rect.r : b.rect.l + p.x * (b.rect.r - b.rect.l),
              p.y == 1.0 ? b.rect.b : b.rect.t + p.y * (b.rect.b - b.rect.t)};
    }
  }
  return {(b.rect.l + b.rect.r) * 0.5, (b.rect.t + b.rect.b) * 0.5};
}

localroute::Rect PointsBounds(const std::vector<localroute::Point>& pts) {
  if (pts.empty()) {
    return {0, 0, 0, 0};
  }
  double l = pts.front().x;
  double r = l;
  double t = pts.front().y;
  double b = t;
  for (const auto& p : pts) {
    l = std::min(l, p.x);
    r = std::max(r, p.x);
    t = std::min(t, p.y);
    b = std::max(b, p.y);
  }
  return {l, t, r, b};
}

enum class PinSide { kLeft = 0, kRight = 1, kTop = 2, kBottom = 3 };

PinSide ClassifyPinSide(const localroute::Pin& p) {
  if (p.dirs & localroute::kLeft) {
    return PinSide::kLeft;
  }
  if (p.dirs & localroute::kRight) {
    return PinSide::kRight;
  }
  if (p.dirs & localroute::kUp) {
    return PinSide::kTop;
  }
  if (p.dirs & localroute::kDown) {
    return PinSide::kBottom;
  }
  if (p.x <= 0.1) {
    return PinSide::kLeft;
  }
  if (p.x >= 0.9) {
    return PinSide::kRight;
  }
  return p.y <= 0.5 ? PinSide::kTop : PinSide::kBottom;
}

std::vector<localroute::Pin> BuildPinsForCounts(int left, int right, int top,
                                                int bottom) {
  std::vector<localroute::Pin> pins;
  pins.reserve(static_cast<std::size_t>(left + right + top + bottom));
  for (int i = 0; i < left; ++i) {
    pins.push_back({static_cast<localroute::Id>(100 + i), 0.0,
                    static_cast<double>(i + 1) / (left + 1),
                    localroute::kLeft});
  }
  for (int i = 0; i < right; ++i) {
    pins.push_back({static_cast<localroute::Id>(200 + i), 1.0,
                    static_cast<double>(i + 1) / (right + 1),
                    localroute::kRight});
  }
  for (int i = 0; i < top; ++i) {
    pins.push_back({static_cast<localroute::Id>(300 + i),
                    static_cast<double>(i + 1) / (top + 1), 0.0,
                    localroute::kUp});
  }
  for (int i = 0; i < bottom; ++i) {
    pins.push_back({static_cast<localroute::Id>(400 + i),
                    static_cast<double>(i + 1) / (bottom + 1), 1.0,
                    localroute::kDown});
  }
  return pins;
}

double PointToSegDistSq(QPointF p, QPointF a, QPointF b) {
  const double vx = b.x() - a.x();
  const double vy = b.y() - a.y();
  const double len_sq = vx * vx + vy * vy;
  if (len_sq <= 1e-9) {
    const double dx = p.x() - a.x();
    const double dy = p.y() - a.y();
    return dx * dx + dy * dy;
  }
  const double t = std::clamp(
      ((p.x() - a.x()) * vx + (p.y() - a.y()) * vy) / len_sq, 0.0, 1.0);
  const double px = a.x() + t * vx;
  const double py = a.y() + t * vy;
  const double dx = p.x() - px;
  const double dy = p.y() - py;
  return dx * dx + dy * dy;
}

}  // namespace

CanvasWidget::CanvasWidget(QWidget* parent) : QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  setAcceptDrops(true);
  setAutoFillBackground(false);
  fps_clock_.start();

  async_timer_.setInterval(16);
  connect(&async_timer_, &QTimer::timeout, this, &CanvasWidget::OnAsyncTick);
}

CanvasWidget::~CanvasWidget() {
  async_timer_.stop();
  async_router_.reset();
  sync_router_.reset();
}

const localroute::LocalOrthogonalRouter& CanvasWidget::ActiveView() const {
  return (mode_ == RouteMode::kAsync && async_router_) ? async_router_->view()
                                                       : *sync_router_;
}

void CanvasWidget::RefreshWorldBounds() {
  const auto& blocks = ActiveView().all_blocks();
  if (blocks.empty()) {
    world_bounds_ = {-400.0, -300.0, 400.0, 300.0};
    return;
  }
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto& kv : blocks) {
    const auto& r = kv.second.rect;
    min_x = std::min(min_x, r.l);
    min_y = std::min(min_y, r.t);
    max_x = std::max(max_x, r.r);
    max_y = std::max(max_y, r.b);
  }
  world_bounds_ = {min_x, min_y, max_x, max_y};
}

bool CanvasWidget::SafePollAsync(std::size_t max_results,
                                 std::vector<localroute::Id>* updated_ids,
                                 std::size_t* polled_count) {
  if (polled_count) {
    *polled_count = 0;
  }
  if (!async_router_) {
    return false;
  }
  try {
    const std::size_t n = async_router_->Poll(max_results, updated_ids);
    if (polled_count) {
      *polled_count = n;
    }
    return true;
  } catch (const std::exception& e) {
    async_timer_.stop();
    qWarning("AsyncLocalRouter::Poll failed: %s", e.what());
  } catch (...) {
    async_timer_.stop();
    qWarning("AsyncLocalRouter::Poll failed with unknown exception");
  }
  return false;
}

void CanvasWidget::PushUndoCommand(EditCommand cmd) {
  redo_stack_.clear();
  undo_stack_.push_back(std::move(cmd));
  while (undo_stack_.size() > 100) {
    undo_stack_.pop_front();
  }
}

void CanvasWidget::ApplyBlockMovesDelta(
    const std::vector<std::pair<localroute::Id, localroute::Rect>>& moves) {
  if (moves.empty()) {
    return;
  }
  std::vector<localroute::Id> ids;
  ids.reserve(moves.size());
  for (const auto& m : moves) {
    ids.push_back(m.first);
  }
  if (mode_ == RouteMode::kSync && sync_router_) {
    sync_router_->MoveBlocksBatch(moves);
    const auto changed = sync_router_->RerouteAffected(ids, true);
    UpdateRouteCacheForIds(changed);
  } else if (mode_ == RouteMode::kAsync && async_router_) {
    async_router_->Move(moves);
    std::vector<localroute::Id> updated;
    const bool poll_ok = SafePollAsync(128, &updated);
    for (localroute::Id id : ids) {
      UpdateRouteBoundsFromBlock(id);
    }
    if (!updated.empty()) {
      UpdateRouteCacheForIds(updated);
    }
    if (poll_ok && !async_timer_.isActive() &&
        async_router_->PendingCount() > 0) {
      async_timer_.start();
    }
  }
}

void CanvasWidget::AddBlockIncremental(localroute::Block blk) {
  if (mode_ == RouteMode::kSync && sync_router_) {
    sync_router_->AddBlock(std::move(blk));
    const auto changed = sync_router_->RerouteAffected({});
    UpdateRouteCacheForIds(changed);
  } else if (mode_ == RouteMode::kAsync && async_router_) {
    async_router_->AddBlock(std::move(blk));
    if (!async_timer_.isActive() && async_router_->PendingCount() > 0) {
      async_timer_.start();
    }
  }
}

void CanvasWidget::RemoveBlockIncremental(localroute::Id bid) {
  std::vector<localroute::Id> removed_conns;
  if (ActiveView().all_blocks().count(bid)) {
    removed_conns = ActiveView().adjacent_conns(bid);
  }
  if (mode_ == RouteMode::kSync && sync_router_) {
    if (sync_router_->all_blocks().count(bid)) {
      sync_router_->RemoveBlock(bid);
    }
  } else if (mode_ == RouteMode::kAsync && async_router_) {
    if (async_router_->view().all_blocks().count(bid)) {
      async_router_->RemoveBlock(bid);
    }
  }
  for (localroute::Id cid : removed_conns) {
    RemoveRouteCacheEntry(cid);
  }
}

void CanvasWidget::AddConnectorIncremental(localroute::Conn conn,
                                           const localroute::Route* opt_route) {
  const localroute::Id cid = conn.id;
  if (mode_ == RouteMode::kSync && sync_router_) {
    sync_router_->AddConnector(conn);
    if (opt_route != nullptr &&
        opt_route->status != localroute::Status::kPending &&
        !opt_route->points.empty()) {
      sync_router_->RestoreRoute(cid, *opt_route);
    } else {
      sync_router_->RouteConnector(cid);
    }
  } else if (mode_ == RouteMode::kAsync && async_router_) {
    async_router_->AddConnector(conn);
    if (opt_route != nullptr &&
        opt_route->status != localroute::Status::kPending &&
        !opt_route->points.empty()) {
      async_router_->RestoreRoute(cid, *opt_route);
    } else if (!async_timer_.isActive() && async_router_->PendingCount() > 0) {
      async_timer_.start();
    }
  }
  AddRouteCacheEntry(cid);
}

void CanvasWidget::RemoveConnectorIncremental(localroute::Id cid) {
  if (mode_ == RouteMode::kSync && sync_router_) {
    if (sync_router_->all_conns().count(cid)) {
      sync_router_->RemoveConnector(cid);
    }
  } else if (mode_ == RouteMode::kAsync && async_router_) {
    if (async_router_->view().all_conns().count(cid)) {
      async_router_->RemoveConnector(cid);
    }
  }
  RemoveRouteCacheEntry(cid);
}

void CanvasWidget::RefreshAfterIncrementalEdit() {
  const auto& view = ActiveView();
  current_block_count_ = static_cast<unsigned>(view.all_blocks().size());
  current_conn_count_ = static_cast<unsigned>(view.all_conns().size());
  RefreshWorldBounds();
  if (mode_ == RouteMode::kAsync && async_router_ &&
      async_router_->PendingCount() > 0 && !async_timer_.isActive()) {
    async_timer_.start();
  }
  update();
  emit StatusChanged();
  emit SelectionChanged();
}

void CanvasWidget::InstallInitialRouter(
    localroute::LocalOrthogonalRouter initial) {
  undo_stack_.clear();
  redo_stack_.clear();
  ctrl_dup_armed_ = false;
  has_pending_dup_cmd_ = false;
  seg_drag_before_routes_.clear();

  current_block_count_ = static_cast<unsigned>(initial.all_blocks().size());
  current_conn_count_ = static_cast<unsigned>(initial.all_conns().size());

  QElapsedTimer timer;
  timer.start();
  if (mode_ == RouteMode::kSync) {
    sync_router_ =
        std::make_unique<localroute::LocalOrthogonalRouter>(std::move(initial));
    sync_router_->RouteAllParallel();
  } else {
    async_router_ =
        std::make_unique<localroute::AsyncLocalRouter>(std::move(initial));
  }
  double setup_ms = static_cast<double>(timer.nsecsElapsed()) / 1e6;

  RefreshWorldBounds();
  RebuildRouteCache();
  if (current_block_count_ > 0 && current_block_count_ <= 32) {
    zoom_ = 1.0;
    const double cx = (world_bounds_.l + world_bounds_.r) * 0.5;
    const double cy = (world_bounds_.t + world_bounds_.b) * 0.5;
    pan_offset_ =
        QPointF(width() * 0.5 - cx * zoom_, height() * 0.5 - cy * zoom_);
    UpdateViewportPriority();
  } else {
    ResetLocalView();
  }

  if (mode_ == RouteMode::kAsync && async_router_) {
    QElapsedTimer start_timer;
    start_timer.start();
    async_router_->Prioritize(WorldViewportRect(), {});
    async_router_->StartAll();
    setup_ms += static_cast<double>(start_timer.nsecsElapsed()) / 1e6;
    RebuildRouteCache();
    if (async_router_->PendingCount() > 0) {
      async_timer_.start();
    }
  }

  route_time_ms_ = setup_ms;
  RecordRouteSample(setup_ms);
  update();
  emit StatusChanged();
  emit SelectionChanged();
}

void CanvasWidget::LoadTopology(unsigned block_count) {
  async_timer_.stop();
  async_router_.reset();
  sync_router_.reset();
  selected_blocks_.clear();
  highlighted_conns_.clear();
  has_selected_conn_ = false;
  selected_conn_id_ = 0;
  hover_pin_ = PinHit{};
  wire_src_ = PinHit{};
  active_guides_.clear();
  drag_seg_conn_id_ = 0;
  drag_seg_index_ = 0;
  ctrl_dup_armed_ = false;
  has_pending_dup_cmd_ = false;
  seg_drag_before_routes_.clear();
  undo_stack_.clear();
  redo_stack_.clear();
  route_samples_ms_.clear();
  drag_kind_ = DragKind::kNone;

  localroute::Config cfg;
  cfg.reroute_third_party_blocked = third_party_reroute_;
  cfg.enable_channel_nudging = channel_nudging_;
  localroute::LocalOrthogonalRouter initial(cfg);

  next_block_id_ = 1;
  next_conn_id_ = 1;
  if (block_count > 0) {
    const localroute::Topology top = localroute::GenerateTopology(block_count);
    for (const auto& b : top.blocks) {
      initial.AddBlock(b);
      next_block_id_ = std::max(next_block_id_, b.id + 1);
    }
    for (const auto& c : top.conns) {
      initial.AddConnector(c);
      next_conn_id_ = std::max(next_conn_id_, c.id + 1);
    }
  }
  InstallInitialRouter(std::move(initial));
}

void CanvasWidget::LoadPresetParallelBundle() {
  LoadTopology(0);
  async_timer_.stop();
  async_router_.reset();
  sync_router_.reset();

  localroute::Config cfg;
  cfg.reroute_third_party_blocked = third_party_reroute_;
  cfg.enable_channel_nudging = channel_nudging_;
  localroute::LocalOrthogonalRouter initial(cfg);

  std::vector<localroute::Pin> b1_pins;
  std::vector<localroute::Pin> b2_pins;
  for (int i = 0; i < 10; ++i) {
    const double y = static_cast<double>(i + 1) / 11.0;
    b1_pins.push_back(
        {static_cast<localroute::Id>(200 + i), 1.0, y, localroute::kRight});
    b2_pins.push_back(
        {static_cast<localroute::Id>(100 + i), 0.0, y, localroute::kLeft});
  }
  initial.AddBlock({1, {0.0, 0.0, 90.0, 260.0}, b1_pins});
  initial.AddBlock({2, {350.0, 60.0, 440.0, 320.0}, b2_pins});
  for (int i = 0; i < 10; ++i) {
    initial.AddConnector({static_cast<localroute::Id>(1 + i), 1,
                          static_cast<localroute::Id>(200 + i), 2,
                          static_cast<localroute::Id>(100 + i)});
  }
  next_block_id_ = 3;
  next_conn_id_ = 11;
  InstallInitialRouter(std::move(initial));
}

void CanvasWidget::LoadPresetCascade() {
  LoadTopology(0);
  async_timer_.stop();
  async_router_.reset();
  sync_router_.reset();

  localroute::Config cfg;
  cfg.reroute_third_party_blocked = third_party_reroute_;
  cfg.enable_channel_nudging = channel_nudging_;
  localroute::LocalOrthogonalRouter initial(cfg);

  initial.AddBlock(
      {9, {0.0, 0.0, 90.0, 50.0}, {{200, 1.0, 0.5, localroute::kRight}}});
  initial.AddBlock(
      {3, {0.0, 80.0, 90.0, 130.0}, {{200, 1.0, 0.5, localroute::kRight}}});
  initial.AddBlock(
      {8, {0.0, 160.0, 90.0, 210.0}, {{200, 1.0, 0.5, localroute::kRight}}});
  initial.AddBlock({10,
                    {270.0, 150.0, 360.0, 230.0},
                    {{100, 0.0, 0.33, localroute::kLeft},
                     {101, 0.0, 0.67, localroute::kLeft}}});
  initial.AddBlock(
      {11, {270.0, 240.0, 360.0, 290.0}, {{100, 0.0, 0.5, localroute::kLeft}}});
  initial.AddBlock(
      {12, {270.0, 320.0, 360.0, 370.0}, {{100, 0.0, 0.5, localroute::kLeft}}});
  initial.AddConnector({1, 9, 200, 10, 100});
  initial.AddConnector({2, 3, 200, 10, 101});
  initial.AddConnector({3, 8, 200, 11, 100});
  initial.AddConnector({4, 8, 200, 12, 100});
  next_block_id_ = 13;
  next_conn_id_ = 5;
  InstallInitialRouter(std::move(initial));
}

void CanvasWidget::SetRouteMode(RouteMode new_mode) {
  if (mode_ == new_mode) {
    return;
  }
  undo_stack_.clear();
  redo_stack_.clear();
  async_timer_.stop();
  localroute::LocalOrthogonalRouter model = ActiveView().Clone();
  model.set_reroute_third_party_blocked(third_party_reroute_);
  model.set_enable_channel_nudging(channel_nudging_);
  async_router_.reset();
  sync_router_.reset();
  mode_ = new_mode;

  QElapsedTimer timer;
  timer.start();
  if (mode_ == RouteMode::kSync) {
    sync_router_ =
        std::make_unique<localroute::LocalOrthogonalRouter>(std::move(model));
    sync_router_->RouteAllParallel();
  } else {
    async_router_ =
        std::make_unique<localroute::AsyncLocalRouter>(std::move(model));
    async_router_->Prioritize(WorldViewportRect(), std::vector<localroute::Id>(
                                                       selected_blocks_.begin(),
                                                       selected_blocks_.end()));
    async_router_->StartAll();
    if (async_router_->PendingCount() > 0) {
      async_timer_.start();
    }
  }
  route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  RecordRouteSample(route_time_ms_);
  RebuildRouteCache();
  update();
  emit StatusChanged();
}

void CanvasWidget::SetRerouteThirdPartyBlocked(bool enabled) {
  if (third_party_reroute_ == enabled) {
    return;
  }
  third_party_reroute_ = enabled;
  if (mode_ == RouteMode::kSync && sync_router_) {
    sync_router_->set_reroute_third_party_blocked(enabled);
  } else if (mode_ == RouteMode::kAsync && async_router_) {
    async_router_->set_reroute_third_party_blocked(enabled);
  }
  update();
  emit StatusChanged();
}

void CanvasWidget::SetEnableChannelNudging(bool enabled) {
  if (channel_nudging_ == enabled) {
    return;
  }
  channel_nudging_ = enabled;
  undo_stack_.clear();
  redo_stack_.clear();
  if (!enabled) {
    // Rebuild from empty routes so previously nudged middle segments do not
    // persist via zero-displacement translation reuse, while preserving
    // hand-edited (user_modified) routes.
    async_timer_.stop();
    const auto& cur = ActiveView();
    std::vector<std::pair<localroute::Id, localroute::Route>> user_routes;
    for (const auto& kv : cur.all_routes()) {
      if (kv.second.user_modified &&
          kv.second.status != localroute::Status::kPending &&
          kv.second.points.size() >= 2) {
        user_routes.emplace_back(kv.first, kv.second);
      }
    }
    localroute::Config cfg = cur.config();
    cfg.reroute_third_party_blocked = third_party_reroute_;
    cfg.enable_channel_nudging = false;
    localroute::LocalOrthogonalRouter fresh(cfg);
    for (const auto& kv : cur.all_blocks()) {
      fresh.AddBlock(kv.second);
    }
    for (const auto& kv : cur.all_conns()) {
      fresh.AddConnector(kv.second);
    }
    async_router_.reset();
    sync_router_.reset();

    QElapsedTimer timer;
    timer.start();
    if (mode_ == RouteMode::kSync) {
      sync_router_ =
          std::make_unique<localroute::LocalOrthogonalRouter>(std::move(fresh));
      sync_router_->RouteAllParallel();
      for (const auto& kv : user_routes) {
        sync_router_->RestoreRoute(kv.first, kv.second);
      }
      route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
    } else {
      async_router_ =
          std::make_unique<localroute::AsyncLocalRouter>(std::move(fresh));
      async_router_->Prioritize(
          WorldViewportRect(),
          std::vector<localroute::Id>(selected_blocks_.begin(),
                                      selected_blocks_.end()));
      async_router_->StartAll();
      for (const auto& kv : user_routes) {
        async_router_->RestoreRoute(kv.first, kv.second);
      }
      route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
      if (async_router_->PendingCount() > 0) {
        async_timer_.start();
      }
    }
    RecordRouteSample(route_time_ms_);
    RebuildRouteCache();
    update();
    emit StatusChanged();
    return;
  }

  if (mode_ == RouteMode::kSync && sync_router_) {
    sync_router_->set_enable_channel_nudging(true);
  } else if (mode_ == RouteMode::kAsync && async_router_) {
    async_router_->set_enable_channel_nudging(true);
  }
  RouteAll();
}

void CanvasWidget::SetHighlightFallback(bool enabled) {
  if (highlight_fallback_ == enabled) {
    return;
  }
  highlight_fallback_ = enabled;
  update();
}

void CanvasWidget::SetShowLineHops(bool enabled) {
  if (show_line_hops_ == enabled) {
    return;
  }
  show_line_hops_ = enabled;
  update();
}

std::vector<localroute::Id> CanvasWidget::AutoRouteSelection() {
  std::vector<localroute::Id> targets;
  const auto& view = ActiveView();
  if (has_selected_conn_ && view.all_conns().count(selected_conn_id_)) {
    targets.push_back(selected_conn_id_);
  }
  for (localroute::Id bid : selected_blocks_) {
    for (localroute::Id cid : view.adjacent_conns(bid)) {
      targets.push_back(cid);
    }
  }
  if (targets.empty()) {
    return {};
  }
  std::sort(targets.begin(), targets.end());
  targets.erase(std::unique(targets.begin(), targets.end()), targets.end());

  EditCommand cmd;
  cmd.kind = EditCommand::Kind::kModifyRoutes;
  const auto& prev_routes = view.all_routes();
  for (localroute::Id cid : targets) {
    if (const auto it = prev_routes.find(cid); it != prev_routes.end()) {
      cmd.before_routes.emplace_back(cid, it->second);
    }
  }

  QElapsedTimer timer;
  timer.start();
  std::vector<localroute::Id> changed;
  if (mode_ == RouteMode::kSync && sync_router_) {
    changed = sync_router_->AutoRouteConnectors(targets);
  } else if (mode_ == RouteMode::kAsync && async_router_) {
    changed = async_router_->AutoRouteConnectors(targets);
  }
  route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  RecordRouteSample(route_time_ms_);

  // Capture any extra nudged connections in before_routes (if missing, record
  // changed routes cleanly so Undo restores every modified route).
  const auto& new_routes = ActiveView().all_routes();
  for (localroute::Id cid : changed) {
    if (const auto it = new_routes.find(cid); it != new_routes.end()) {
      cmd.after_routes.emplace_back(cid, it->second);
    }
  }
  if (!cmd.before_routes.empty() && !cmd.after_routes.empty()) {
    PushUndoCommand(std::move(cmd));
  }

  UpdateRouteCacheForIds(changed);
  update();
  emit StatusChanged();
  return changed;
}

bool CanvasWidget::ExportViewportSvg(const QString& path) const {
  if (path.isEmpty()) {
    return false;
  }
  try {
    std::vector<localroute::Id> hi(highlighted_conns_.begin(),
                                   highlighted_conns_.end());
    ActiveView().ExportRegionToSvg(path.toStdString(), WorldViewportRect(), hi);
    return true;
  } catch (...) {
    return false;
  }
}

bool CanvasWidget::Undo() {
  if (undo_stack_.empty()) {
    return false;
  }
  EditCommand cmd = std::move(undo_stack_.back());
  undo_stack_.pop_back();

  QElapsedTimer timer;
  timer.start();
  const auto& view = ActiveView();
  switch (cmd.kind) {
    case EditCommand::Kind::kMoveBlocks: {
      std::vector<std::pair<localroute::Id, localroute::Rect>> live;
      for (const auto& kv : cmd.before_rects) {
        if (view.all_blocks().count(kv.first)) {
          live.push_back(kv);
        }
      }
      ApplyBlockMovesDelta(live);
      break;
    }
    case EditCommand::Kind::kAddItems: {
      if (cmd.after_routes.empty()) {
        for (const auto& c : cmd.conns) {
          if (const auto rit = view.all_routes().find(c.id);
              rit != view.all_routes().end()) {
            cmd.after_routes.emplace_back(c.id, rit->second);
          }
        }
      }
      for (const auto& c : cmd.conns) {
        selected_conn_id_ = (selected_conn_id_ == c.id) ? 0 : selected_conn_id_;
        highlighted_conns_.erase(c.id);
        RemoveConnectorIncremental(c.id);
      }
      has_selected_conn_ = highlighted_conns_.count(selected_conn_id_) != 0;
      for (const auto& b : cmd.blocks) {
        selected_blocks_.erase(b.id);
        RemoveBlockIncremental(b.id);
      }
      break;
    }
    case EditCommand::Kind::kDeleteItems: {
      for (const auto& b : cmd.blocks) {
        AddBlockIncremental(b);
      }
      std::unordered_map<localroute::Id, const localroute::Route*> saved;
      for (const auto& kv : cmd.before_routes) {
        saved.emplace(kv.first, &kv.second);
      }
      for (const auto& c : cmd.conns) {
        const auto sit = saved.find(c.id);
        AddConnectorIncremental(c, sit != saved.end() ? sit->second : nullptr);
      }
      break;
    }
    case EditCommand::Kind::kUpdatePins: {
      if (view.all_blocks().count(cmd.pin_block_id)) {
        std::vector<localroute::Id> surviving;
        if (mode_ == RouteMode::kSync && sync_router_) {
          surviving = sync_router_->UpdateBlockPins(cmd.pin_block_id,
                                                    cmd.old_rect, cmd.old_pins);
        } else if (mode_ == RouteMode::kAsync && async_router_) {
          surviving = async_router_->UpdateBlockPins(
              cmd.pin_block_id, cmd.old_rect, cmd.old_pins);
        }
        std::unordered_map<localroute::Id, const localroute::Route*> saved;
        for (const auto& kv : cmd.before_routes) {
          saved.emplace(kv.first, &kv.second);
        }
        for (const auto& c : cmd.conns) {
          const auto sit = saved.find(c.id);
          AddConnectorIncremental(c,
                                  sit != saved.end() ? sit->second : nullptr);
        }
        std::vector<localroute::Id> restored_ids = surviving;
        for (localroute::Id cid : surviving) {
          if (const auto sit = saved.find(cid);
              sit != saved.end() &&
              sit->second->status != localroute::Status::kPending &&
              !sit->second->points.empty()) {
            if (mode_ == RouteMode::kSync && sync_router_) {
              sync_router_->RestoreRoute(cid, *sit->second);
            } else if (mode_ == RouteMode::kAsync && async_router_) {
              async_router_->RestoreRoute(cid, *sit->second);
            }
          } else if (mode_ == RouteMode::kSync && sync_router_) {
            sync_router_->RouteConnector(cid);
          }
        }
        UpdateRouteCacheForIds(restored_ids);
      }
      break;
    }
    case EditCommand::Kind::kModifyRoutes: {
      std::vector<localroute::Id> ids;
      ids.reserve(cmd.before_routes.size());
      for (const auto& kv : cmd.before_routes) {
        if (!ActiveView().all_conns().count(kv.first)) {
          continue;
        }
        if (mode_ == RouteMode::kSync && sync_router_) {
          sync_router_->RestoreRoute(kv.first, kv.second);
        } else if (mode_ == RouteMode::kAsync && async_router_) {
          async_router_->RestoreRoute(kv.first, kv.second);
        }
        ids.push_back(kv.first);
      }
      UpdateRouteCacheForIds(ids);
      break;
    }
  }

  route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  RecordRouteSample(route_time_ms_);
  redo_stack_.push_back(std::move(cmd));
  RefreshAfterIncrementalEdit();
  return true;
}

bool CanvasWidget::Redo() {
  if (redo_stack_.empty()) {
    return false;
  }
  EditCommand cmd = std::move(redo_stack_.back());
  redo_stack_.pop_back();

  QElapsedTimer timer;
  timer.start();
  const auto& view = ActiveView();
  switch (cmd.kind) {
    case EditCommand::Kind::kMoveBlocks: {
      std::vector<std::pair<localroute::Id, localroute::Rect>> live;
      for (const auto& kv : cmd.after_rects) {
        if (view.all_blocks().count(kv.first)) {
          live.push_back(kv);
        }
      }
      ApplyBlockMovesDelta(live);
      break;
    }
    case EditCommand::Kind::kAddItems: {
      for (const auto& b : cmd.blocks) {
        AddBlockIncremental(b);
      }
      std::unordered_map<localroute::Id, const localroute::Route*> saved;
      for (const auto& kv : cmd.after_routes) {
        saved.emplace(kv.first, &kv.second);
      }
      for (const auto& c : cmd.conns) {
        const auto sit = saved.find(c.id);
        AddConnectorIncremental(c, sit != saved.end() ? sit->second : nullptr);
      }
      break;
    }
    case EditCommand::Kind::kDeleteItems: {
      for (const auto& c : cmd.conns) {
        RemoveConnectorIncremental(c.id);
      }
      for (const auto& b : cmd.blocks) {
        selected_blocks_.erase(b.id);
        RemoveBlockIncremental(b.id);
      }
      break;
    }
    case EditCommand::Kind::kUpdatePins: {
      if (view.all_blocks().count(cmd.pin_block_id)) {
        std::vector<localroute::Id> surviving;
        if (mode_ == RouteMode::kSync && sync_router_) {
          surviving = sync_router_->UpdateBlockPins(cmd.pin_block_id,
                                                    cmd.new_rect, cmd.new_pins);
          const auto changed =
              sync_router_->RerouteAffected({cmd.pin_block_id});
          UpdateRouteCacheForIds(changed);
        } else if (mode_ == RouteMode::kAsync && async_router_) {
          surviving = async_router_->UpdateBlockPins(
              cmd.pin_block_id, cmd.new_rect, cmd.new_pins);
          UpdateRouteCacheForIds(surviving);
        }
        for (const auto& c : cmd.conns) {
          RemoveRouteCacheEntry(c.id);
        }
      }
      break;
    }
    case EditCommand::Kind::kModifyRoutes: {
      std::vector<localroute::Id> ids;
      ids.reserve(cmd.after_routes.size());
      for (const auto& kv : cmd.after_routes) {
        if (!ActiveView().all_conns().count(kv.first)) {
          continue;
        }
        if (mode_ == RouteMode::kSync && sync_router_) {
          sync_router_->RestoreRoute(kv.first, kv.second);
        } else if (mode_ == RouteMode::kAsync && async_router_) {
          async_router_->RestoreRoute(kv.first, kv.second);
        }
        ids.push_back(kv.first);
      }
      UpdateRouteCacheForIds(ids);
      break;
    }
  }

  route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  RecordRouteSample(route_time_ms_);
  undo_stack_.push_back(std::move(cmd));
  while (undo_stack_.size() > 100) {
    undo_stack_.pop_front();
  }
  RefreshAfterIncrementalEdit();
  return true;
}

void CanvasWidget::RouteAll() {
  QElapsedTimer timer;
  timer.start();
  if (mode_ == RouteMode::kSync && sync_router_) {
    sync_router_->RouteAllParallel();
    route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  } else if (mode_ == RouteMode::kAsync && async_router_) {
    async_router_->Prioritize(WorldViewportRect(), std::vector<localroute::Id>(
                                                       selected_blocks_.begin(),
                                                       selected_blocks_.end()));
    async_router_->StartAll();
    route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
    if (async_router_->PendingCount() > 0) {
      async_timer_.start();
    }
  }
  RecordRouteSample(route_time_ms_);
  RebuildRouteCache();
  update();
  emit StatusChanged();
}

void CanvasWidget::FitAll() {
  RefreshWorldBounds();
  const double bw = std::max(1.0, world_bounds_.r - world_bounds_.l);
  const double bh = std::max(1.0, world_bounds_.b - world_bounds_.t);
  const double vw = std::max(100.0, static_cast<double>(width()) - 40.0);
  const double vh = std::max(100.0, static_cast<double>(height()) - 40.0);
  zoom_ = std::clamp(std::min(vw / bw, vh / bh), 0.05, 4.0);
  const double cx = (world_bounds_.l + world_bounds_.r) * 0.5;
  const double cy = (world_bounds_.t + world_bounds_.b) * 0.5;
  pan_offset_ =
      QPointF(width() * 0.5 - cx * zoom_, height() * 0.5 - cy * zoom_);
  UpdateViewportPriority();
  update();
}

void CanvasWidget::ResetLocalView() {
  zoom_ = 1.0;
  const auto& view = ActiveView();
  const auto& blocks = view.all_blocks();
  if (!blocks.empty()) {
    const localroute::Id mid_id = current_block_count_ / 2;
    auto it = blocks.find(mid_id);
    if (it == blocks.end()) {
      it = blocks.begin();
    }
    const auto& r = it->second.rect;
    const double cx = (r.l + r.r) * 0.5;
    const double cy = (r.t + r.b) * 0.5;
    pan_offset_ =
        QPointF(width() * 0.5 - cx * zoom_, height() * 0.5 - cy * zoom_);
  } else {
    pan_offset_ = QPointF(width() * 0.5, height() * 0.5);
  }
  UpdateViewportPriority();
  update();
}

localroute::Id CanvasWidget::AddBlockFromTemplate(const BlockTemplate& tmpl,
                                                  QPointF world_center) {
  const localroute::Id new_id = next_block_id_++;
  const double min_h = std::max(
      tmpl.height, std::max(tmpl.left_pins, tmpl.right_pins) * 20.0 + 16.0);
  const double min_w = std::max(
      tmpl.width, std::max(tmpl.top_pins, tmpl.bottom_pins) * 24.0 + 20.0);
  const localroute::Rect rect{
      world_center.x() - min_w * 0.5, world_center.y() - min_h * 0.5,
      world_center.x() + min_w * 0.5, world_center.y() + min_h * 0.5};
  localroute::Block blk{new_id, rect,
                        BuildPinsForCounts(tmpl.left_pins, tmpl.right_pins,
                                           tmpl.top_pins, tmpl.bottom_pins)};

  selected_blocks_.clear();
  selected_blocks_.insert(new_id);
  has_selected_conn_ = false;
  highlighted_conns_.clear();

  QElapsedTimer timer;
  timer.start();
  EditCommand cmd;
  cmd.kind = EditCommand::Kind::kAddItems;
  cmd.blocks.push_back(blk);
  AddBlockIncremental(std::move(blk));
  route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  RecordRouteSample(route_time_ms_);
  PushUndoCommand(std::move(cmd));
  RefreshAfterIncrementalEdit();
  return new_id;
}

localroute::Id CanvasWidget::SpawnBlockAtCenter(const BlockTemplate& tmpl) {
  QPointF center = ScreenToWorld(QPointF(width() * 0.5, height() * 0.5));
  const double jitter = static_cast<double>((next_block_id_ % 5) * 18) - 36.0;
  center += QPointF(jitter, jitter);
  return AddBlockFromTemplate(tmpl, center);
}

bool CanvasWidget::GetSelectedBlockPorts(localroute::Id* block_id, int* left,
                                         int* right, int* top,
                                         int* bottom) const {
  if (selected_blocks_.size() != 1) {
    return false;
  }
  const localroute::Id id = *selected_blocks_.begin();
  const auto& blocks = ActiveView().all_blocks();
  const auto it = blocks.find(id);
  if (it == blocks.end()) {
    return false;
  }
  int l = 0;
  int r = 0;
  int t = 0;
  int b = 0;
  for (const auto& p : it->second.pins) {
    switch (ClassifyPinSide(p)) {
      case PinSide::kLeft:
        ++l;
        break;
      case PinSide::kRight:
        ++r;
        break;
      case PinSide::kTop:
        ++t;
        break;
      case PinSide::kBottom:
        ++b;
        break;
    }
  }
  if (block_id) {
    *block_id = id;
  }
  if (left) {
    *left = l;
  }
  if (right) {
    *right = r;
  }
  if (top) {
    *top = t;
  }
  if (bottom) {
    *bottom = b;
  }
  return true;
}

void CanvasWidget::SetSelectedBlockPorts(int left, int right, int top,
                                         int bottom) {
  if (selected_blocks_.size() != 1) {
    return;
  }
  SetBlockPorts(*selected_blocks_.begin(), left, right, top, bottom);
}

void CanvasWidget::SetBlockPorts(localroute::Id block_id, int left, int right,
                                 int top, int bottom) {
  left = std::clamp(left, 0, 16);
  right = std::clamp(right, 0, 16);
  top = std::clamp(top, 0, 8);
  bottom = std::clamp(bottom, 0, 8);

  const auto& view = ActiveView();
  if (!view.all_blocks().count(block_id)) {
    return;
  }

  const localroute::Block old_blk = view.block(block_id);
  const double cur_w = old_blk.rect.r - old_blk.rect.l;
  const double cur_h = old_blk.rect.b - old_blk.rect.t;
  const double cx = (old_blk.rect.l + old_blk.rect.r) * 0.5;
  const double cy = (old_blk.rect.t + old_blk.rect.b) * 0.5;
  const double new_h = std::max(cur_h, std::max(left, right) * 20.0 + 16.0);
  const double new_w = std::max(cur_w, std::max(top, bottom) * 24.0 + 20.0);
  const localroute::Rect new_rect{cx - new_w * 0.5, cy - new_h * 0.5,
                                  cx + new_w * 0.5, cy + new_h * 0.5};
  std::vector<localroute::Pin> new_pins =
      BuildPinsForCounts(left, right, top, bottom);
  std::unordered_set<localroute::Id> new_pin_ids;
  for (const auto& p : new_pins) {
    new_pin_ids.insert(p.id);
  }

  EditCommand cmd;
  cmd.kind = EditCommand::Kind::kUpdatePins;
  cmd.pin_block_id = block_id;
  cmd.old_rect = old_blk.rect;
  cmd.new_rect = new_rect;
  cmd.old_pins = old_blk.pins;
  cmd.new_pins = new_pins;

  std::vector<localroute::Id> removed_cids;
  const auto& all_conns = view.all_conns();
  const auto& all_routes = view.all_routes();
  for (localroute::Id cid : view.adjacent_conns(block_id)) {
    const auto cit = all_conns.find(cid);
    if (cit == all_conns.end()) {
      continue;
    }
    const localroute::Conn& c = cit->second;
    if (const auto rit = all_routes.find(cid); rit != all_routes.end()) {
      cmd.before_routes.emplace_back(cid, rit->second);
    }
    if ((c.src == block_id && !new_pin_ids.count(c.sp)) ||
        (c.dst == block_id && !new_pin_ids.count(c.dp))) {
      cmd.conns.push_back(c);
      removed_cids.push_back(cid);
    }
  }

  QElapsedTimer timer;
  timer.start();
  if (mode_ == RouteMode::kSync && sync_router_) {
    (void)sync_router_->UpdateBlockPins(block_id, new_rect,
                                        std::move(new_pins));
    const auto changed = sync_router_->RerouteAffected({block_id});
    UpdateRouteCacheForIds(changed);
  } else if (mode_ == RouteMode::kAsync && async_router_) {
    const auto surviving =
        async_router_->UpdateBlockPins(block_id, new_rect, std::move(new_pins));
    UpdateRouteCacheForIds(surviving);
  }
  for (localroute::Id cid : removed_cids) {
    RemoveRouteCacheEntry(cid);
  }
  route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  RecordRouteSample(route_time_ms_);
  PushUndoCommand(std::move(cmd));

  highlighted_conns_.clear();
  for (localroute::Id cid : ActiveView().adjacent_conns(block_id)) {
    highlighted_conns_.insert(cid);
  }
  RefreshAfterIncrementalEdit();
}

localroute::Id CanvasWidget::AddWire(localroute::Id src_block,
                                     localroute::Id src_pin,
                                     localroute::Id dst_block,
                                     localroute::Id dst_pin) {
  const localroute::Id new_cid = next_conn_id_++;
  const localroute::Conn conn{new_cid, src_block, src_pin, dst_block, dst_pin};
  has_selected_conn_ = true;
  selected_conn_id_ = new_cid;
  highlighted_conns_.clear();
  highlighted_conns_.insert(new_cid);

  QElapsedTimer timer;
  timer.start();
  AddConnectorIncremental(conn);
  route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  RecordRouteSample(route_time_ms_);

  EditCommand cmd;
  cmd.kind = EditCommand::Kind::kAddItems;
  cmd.conns.push_back(conn);
  if (const auto rit = ActiveView().all_routes().find(new_cid);
      rit != ActiveView().all_routes().end()) {
    cmd.after_routes.emplace_back(new_cid, rit->second);
  }
  PushUndoCommand(std::move(cmd));
  RefreshAfterIncrementalEdit();
  return new_cid;
}

void CanvasWidget::DeleteSelection() {
  const auto& view = ActiveView();
  if (has_selected_conn_) {
    const localroute::Id cid = selected_conn_id_;
    has_selected_conn_ = false;
    highlighted_conns_.erase(cid);
    const auto cit = view.all_conns().find(cid);
    if (cit == view.all_conns().end()) {
      return;
    }
    EditCommand cmd;
    cmd.kind = EditCommand::Kind::kDeleteItems;
    cmd.conns.push_back(cit->second);
    if (const auto rit = view.all_routes().find(cid);
        rit != view.all_routes().end()) {
      cmd.before_routes.emplace_back(cid, rit->second);
    }
    QElapsedTimer timer;
    timer.start();
    RemoveConnectorIncremental(cid);
    route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
    RecordRouteSample(route_time_ms_);
    PushUndoCommand(std::move(cmd));
    RefreshAfterIncrementalEdit();
    return;
  }
  if (!selected_blocks_.empty()) {
    const std::vector<localroute::Id> to_del(selected_blocks_.begin(),
                                             selected_blocks_.end());
    EditCommand cmd;
    cmd.kind = EditCommand::Kind::kDeleteItems;
    std::unordered_set<localroute::Id> seen_conns;
    for (localroute::Id bid : to_del) {
      if (!view.all_blocks().count(bid)) {
        continue;
      }
      cmd.blocks.push_back(view.block(bid));
      for (localroute::Id cid : view.adjacent_conns(bid)) {
        if (seen_conns.insert(cid).second && view.all_conns().count(cid)) {
          cmd.conns.push_back(view.all_conns().at(cid));
          if (const auto rit = view.all_routes().find(cid);
              rit != view.all_routes().end()) {
            cmd.before_routes.emplace_back(cid, rit->second);
          }
        }
      }
    }
    selected_blocks_.clear();
    highlighted_conns_.clear();

    QElapsedTimer timer;
    timer.start();
    for (localroute::Id bid : to_del) {
      RemoveBlockIncremental(bid);
    }
    route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
    RecordRouteSample(route_time_ms_);
    if (!cmd.blocks.empty()) {
      PushUndoCommand(std::move(cmd));
    }
    RefreshAfterIncrementalEdit();
  }
}

void CanvasWidget::ClearBlockWires(localroute::Id block_id) {
  const auto& view = ActiveView();
  if (!view.all_blocks().count(block_id)) {
    return;
  }
  const std::vector<localroute::Id> conns = view.adjacent_conns(block_id);
  if (conns.empty()) {
    return;
  }
  EditCommand cmd;
  cmd.kind = EditCommand::Kind::kDeleteItems;
  for (localroute::Id cid : conns) {
    if (view.all_conns().count(cid)) {
      cmd.conns.push_back(view.all_conns().at(cid));
      if (const auto rit = view.all_routes().find(cid);
          rit != view.all_routes().end()) {
        cmd.before_routes.emplace_back(cid, rit->second);
      }
    }
  }
  highlighted_conns_.clear();
  has_selected_conn_ = false;

  QElapsedTimer timer;
  timer.start();
  for (localroute::Id cid : conns) {
    RemoveConnectorIncremental(cid);
  }
  route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  RecordRouteSample(route_time_ms_);
  PushUndoCommand(std::move(cmd));
  RefreshAfterIncrementalEdit();
}

void CanvasWidget::AddRouteCacheEntry(localroute::Id conn_id) {
  if (route_cache_index_.count(conn_id)) {
    UpdateRouteCacheForIds({conn_id});
    return;
  }
  const auto& view = ActiveView();
  const auto cit = view.all_conns().find(conn_id);
  if (cit == view.all_conns().end()) {
    return;
  }
  const auto& c = cit->second;
  const auto& blocks = view.all_blocks();
  const auto& routes = view.all_routes();
  CachedRoute entry;
  entry.id = conn_id;
  const auto rit = routes.find(conn_id);
  if (rit == routes.end() ||
      rit->second.status == localroute::Status::kPending ||
      rit->second.points.empty()) {
    entry.status = localroute::Status::kPending;
    entry.bounds = localroute::Box(PinPos(blocks.at(c.src), c.sp),
                                   PinPos(blocks.at(c.dst), c.dp));
  } else {
    entry.status = rit->second.status;
    entry.bounds = PointsBounds(rit->second.points);
  }
  ++status_counts_[static_cast<std::size_t>(entry.status)];
  route_cache_index_.emplace(conn_id, route_cache_.size());
  route_cache_.push_back(entry);
}

void CanvasWidget::RemoveRouteCacheEntry(localroute::Id conn_id) {
  const auto it = route_cache_index_.find(conn_id);
  if (it == route_cache_index_.end()) {
    return;
  }
  const std::size_t idx = it->second;
  --status_counts_[static_cast<std::size_t>(route_cache_[idx].status)];
  const std::size_t last = route_cache_.size() - 1;
  if (idx != last) {
    route_cache_[idx] = route_cache_[last];
    route_cache_index_[route_cache_[idx].id] = idx;
  }
  route_cache_.pop_back();
  route_cache_index_.erase(it);
}

void CanvasWidget::RebuildRouteCache() {
  const auto& view = ActiveView();
  const auto& conns = view.all_conns();
  const auto& routes = view.all_routes();
  const auto& blocks = view.all_blocks();

  route_cache_.clear();
  route_cache_.reserve(conns.size());
  route_cache_index_.clear();
  route_cache_index_.reserve(conns.size());
  status_counts_.fill(0);

  for (const auto& kv : conns) {
    const auto& c = kv.second;
    CachedRoute entry;
    entry.id = c.id;
    const auto rit = routes.find(c.id);
    if (rit == routes.end() ||
        rit->second.status == localroute::Status::kPending ||
        rit->second.points.empty()) {
      entry.status = localroute::Status::kPending;
      const auto& sb = blocks.at(c.src);
      const auto& db = blocks.at(c.dst);
      entry.bounds = localroute::Box(PinPos(sb, c.sp), PinPos(db, c.dp));
    } else {
      entry.status = rit->second.status;
      entry.bounds = PointsBounds(rit->second.points);
    }
    ++status_counts_[static_cast<std::size_t>(entry.status)];
    route_cache_index_.emplace(c.id, route_cache_.size());
    route_cache_.push_back(entry);
  }
}

void CanvasWidget::UpdateRouteCacheForIds(
    const std::vector<localroute::Id>& ids) {
  const auto& view = ActiveView();
  const auto& routes = view.all_routes();
  const auto& conns = view.all_conns();
  const auto& blocks = view.all_blocks();

  for (localroute::Id id : ids) {
    const auto idx_it = route_cache_index_.find(id);
    if (idx_it == route_cache_index_.end()) {
      continue;
    }
    CachedRoute& entry = route_cache_[idx_it->second];
    --status_counts_[static_cast<std::size_t>(entry.status)];

    const auto rit = routes.find(id);
    if (rit == routes.end() ||
        rit->second.status == localroute::Status::kPending ||
        rit->second.points.empty()) {
      entry.status = localroute::Status::kPending;
      const auto& c = conns.at(id);
      entry.bounds = localroute::Box(PinPos(blocks.at(c.src), c.sp),
                                     PinPos(blocks.at(c.dst), c.dp));
    } else {
      entry.status = rit->second.status;
      entry.bounds = PointsBounds(rit->second.points);
    }
    ++status_counts_[static_cast<std::size_t>(entry.status)];
  }
}

void CanvasWidget::UpdateRouteBoundsFromBlock(localroute::Id block_id) {
  const auto& view = ActiveView();
  const auto& conns = view.all_conns();
  const auto& blocks = view.all_blocks();
  const auto& routes = view.all_routes();
  for (localroute::Id cid : view.adjacent_conns(block_id)) {
    const auto idx_it = route_cache_index_.find(cid);
    if (idx_it == route_cache_index_.end()) {
      continue;
    }
    CachedRoute& entry = route_cache_[idx_it->second];
    --status_counts_[static_cast<std::size_t>(entry.status)];
    const auto& c = conns.at(cid);
    const auto rit = routes.find(cid);
    entry.status = (rit == routes.end()) ? localroute::Status::kPending
                                         : rit->second.status;
    if (entry.status == localroute::Status::kPending ||
        rit->second.points.empty()) {
      entry.bounds = localroute::Box(PinPos(blocks.at(c.src), c.sp),
                                     PinPos(blocks.at(c.dst), c.dp));
    } else {
      entry.bounds = PointsBounds(rit->second.points);
    }
    ++status_counts_[static_cast<std::size_t>(entry.status)];
  }
}

void CanvasWidget::RecordRouteSample(double ms) {
  route_samples_ms_.push_back(ms);
  if (route_samples_ms_.size() > 100) {
    route_samples_ms_.pop_front();
  }
}

double CanvasWidget::RouteP99Ms() const {
  if (route_samples_ms_.empty()) {
    return 0.0;
  }
  std::vector<double> tmp(route_samples_ms_.begin(), route_samples_ms_.end());
  std::sort(tmp.begin(), tmp.end());
  const std::size_t idx =
      static_cast<std::size_t>(std::ceil(0.99 * tmp.size())) - 1;
  return tmp[std::min(idx, tmp.size() - 1)];
}

void CanvasWidget::UpdateViewportPriority() {
  if (mode_ == RouteMode::kAsync && async_router_) {
    async_router_->Prioritize(WorldViewportRect(), std::vector<localroute::Id>(
                                                       selected_blocks_.begin(),
                                                       selected_blocks_.end()));
  }
}

localroute::Rect CanvasWidget::WorldViewportRect() const {
  const QPointF tl = ScreenToWorld(QPointF(0, 0));
  const QPointF br = ScreenToWorld(QPointF(width(), height()));
  return {std::min(tl.x(), br.x()), std::min(tl.y(), br.y()),
          std::max(tl.x(), br.x()), std::max(tl.y(), br.y())};
}

QPointF CanvasWidget::ScreenToWorld(const QPointF& p) const {
  return {(p.x() - pan_offset_.x()) / zoom_, (p.y() - pan_offset_.y()) / zoom_};
}

QPointF CanvasWidget::WorldToScreen(const QPointF& p) const {
  return {p.x() * zoom_ + pan_offset_.x(), p.y() * zoom_ + pan_offset_.y()};
}

localroute::Id CanvasWidget::HitTestBlock(const QPointF& world_pos) const {
  const auto& view = ActiveView();
  const localroute::Rect probe{world_pos.x() - 1.0, world_pos.y() - 1.0,
                               world_pos.x() + 1.0, world_pos.y() + 1.0};
  const auto ids = view.QueryBlocks(probe);
  for (auto it = ids.rbegin(); it != ids.rend(); ++it) {
    const auto& r = view.block(*it).rect;
    if (world_pos.x() >= r.l && world_pos.x() <= r.r && world_pos.y() >= r.t &&
        world_pos.y() <= r.b) {
      return *it;
    }
  }
  return std::numeric_limits<localroute::Id>::max();
}

CanvasWidget::PinHit CanvasWidget::HitTestPin(const QPointF& screen_pos,
                                              double max_px) const {
  if (zoom_ < 0.25) {
    return PinHit{};
  }
  const QPointF world_pos = ScreenToWorld(screen_pos);
  const double pad = max_px / zoom_ + 4.0;
  const localroute::Rect probe{world_pos.x() - pad, world_pos.y() - pad,
                               world_pos.x() + pad, world_pos.y() + pad};
  const auto& view = ActiveView();
  PinHit best;
  double best_dist_sq = max_px * max_px;
  for (localroute::Id bid : view.QueryBlocks(probe)) {
    const auto& b = view.block(bid);
    for (const auto& p : b.pins) {
      const localroute::Point wp = PinPos(b, p.id);
      const QPointF sp = WorldToScreen(QPointF(wp.x, wp.y));
      const double dx = sp.x() - screen_pos.x();
      const double dy = sp.y() - screen_pos.y();
      const double d2 = dx * dx + dy * dy;
      if (d2 <= best_dist_sq) {
        best_dist_sq = d2;
        best = {true, bid, p.id, wp};
      }
    }
  }
  return best;
}

bool CanvasWidget::HitTestWire(const QPointF& screen_pos,
                               localroute::Id* out_conn_id) const {
  const QPointF world_pos = ScreenToWorld(screen_pos);
  const double tol_world = 7.0 / zoom_;
  const double tol_sq = 49.0;  // 7px in screen space
  const localroute::Rect probe{
      world_pos.x() - tol_world, world_pos.y() - tol_world,
      world_pos.x() + tol_world, world_pos.y() + tol_world};
  const auto& view = ActiveView();
  const auto& routes = view.all_routes();
  for (const auto& entry : route_cache_) {
    if (!entry.bounds.Intersects(probe)) {
      continue;
    }
    std::vector<localroute::Point> pts;
    const auto rit = routes.find(entry.id);
    if (rit == routes.end() ||
        rit->second.status == localroute::Status::kPending ||
        rit->second.points.empty()) {
      pts = view.Preview(entry.id);
    } else {
      pts = rit->second.points;
    }
    for (std::size_t j = 1; j < pts.size(); ++j) {
      const QPointF a = WorldToScreen(QPointF(pts[j - 1].x, pts[j - 1].y));
      const QPointF b = WorldToScreen(QPointF(pts[j].x, pts[j].y));
      if (PointToSegDistSq(screen_pos, a, b) <= tol_sq) {
        if (out_conn_id) {
          *out_conn_id = entry.id;
        }
        return true;
      }
    }
  }
  return false;
}

bool CanvasWidget::HitTestWireSegment(const QPointF& screen_pos,
                                      localroute::Id* out_conn_id,
                                      std::size_t* out_seg_index,
                                      bool* out_is_vertical) const {
  if (zoom_ < 0.25) {
    return false;
  }
  const QPointF world_pos = ScreenToWorld(screen_pos);
  const double tol_world = 5.0 / zoom_;
  const double tol_sq = 25.0;  // 5px in screen space
  const localroute::Rect probe{
      world_pos.x() - tol_world, world_pos.y() - tol_world,
      world_pos.x() + tol_world, world_pos.y() + tol_world};
  const auto& view = ActiveView();
  const auto& routes = view.all_routes();

  bool found = false;
  double best_dist_sq = tol_sq + 1.0;
  bool best_is_middle = false;
  localroute::Id best_cid = 0;
  std::size_t best_idx = 0;
  bool best_vert = false;

  for (const auto& entry : route_cache_) {
    if (!entry.bounds.Intersects(probe) ||
        entry.status == localroute::Status::kPending) {
      continue;
    }
    const auto rit = routes.find(entry.id);
    if (rit == routes.end() || rit->second.points.size() < 2) {
      continue;
    }
    const auto& pts = rit->second.points;
    for (std::size_t j = 0; j + 1 < pts.size(); ++j) {
      const localroute::Point& p0 = pts[j];
      const localroute::Point& p1 = pts[j + 1];
      if (std::abs(p0.x - p1.x) <= 1e-6 && std::abs(p0.y - p1.y) <= 1e-6) {
        continue;
      }
      const QPointF a = WorldToScreen(QPointF(p0.x, p0.y));
      const QPointF b = WorldToScreen(QPointF(p1.x, p1.y));
      const double d2 = PointToSegDistSq(screen_pos, a, b);
      if (d2 <= tol_sq) {
        const bool is_middle =
            (pts.size() >= 4 && j >= 1 && j + 2 < pts.size());
        if (!found || d2 < best_dist_sq - 1.0 ||
            (std::abs(d2 - best_dist_sq) <= 1.0 && is_middle &&
             !best_is_middle)) {
          found = true;
          best_dist_sq = d2;
          best_is_middle = is_middle;
          best_cid = entry.id;
          best_idx = j;
          best_vert = std::abs(p0.x - p1.x) <= std::abs(p0.y - p1.y);
        }
      }
    }
  }
  if (!found) {
    return false;
  }
  if (out_conn_id) {
    *out_conn_id = best_cid;
  }
  if (out_seg_index) {
    *out_seg_index = best_idx;
  }
  if (out_is_vertical) {
    *out_is_vertical = best_vert;
  }
  return true;
}

void CanvasWidget::StepSmokeDrag(const std::vector<localroute::Id>& ids,
                                 double dx, double dy) {
  if (ids.empty()) {
    return;
  }
  const auto& view = ActiveView();
  std::vector<std::pair<localroute::Id, localroute::Rect>> before;
  std::vector<std::pair<localroute::Id, localroute::Rect>> moves;
  before.reserve(ids.size());
  moves.reserve(ids.size());
  for (localroute::Id id : ids) {
    const localroute::Rect r = view.block(id).rect;
    before.emplace_back(id, r);
    moves.push_back({id, {r.l + dx, r.t + dy, r.r + dx, r.b + dy}});
  }
  QElapsedTimer timer;
  timer.start();
  ApplyBlockMovesDelta(moves);
  route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  RecordRouteSample(route_time_ms_);

  EditCommand cmd;
  cmd.kind = EditCommand::Kind::kMoveBlocks;
  cmd.before_rects = std::move(before);
  cmd.after_rects = std::move(moves);
  PushUndoCommand(std::move(cmd));
  emit StatusChanged();
}

bool CanvasWidget::StepSmokeSegmentDrag(localroute::Id conn_id,
                                        std::size_t seg_index,
                                        double new_coord) {
  QElapsedTimer timer;
  timer.start();
  bool ok = false;
  std::vector<localroute::Id> changed;
  if (mode_ == RouteMode::kSync && sync_router_) {
    ok =
        sync_router_->MoveRouteSegment(conn_id, seg_index, new_coord, &changed);
  } else if (mode_ == RouteMode::kAsync && async_router_) {
    ok = async_router_->MoveRouteSegment(conn_id, seg_index, new_coord,
                                         &changed);
  }
  route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  RecordRouteSample(route_time_ms_);
  if (ok) {
    UpdateRouteCacheForIds(
        changed.empty() ? std::vector<localroute::Id>{conn_id} : changed);
    update();
    emit StatusChanged();
  }
  return ok;
}

std::vector<localroute::Id> CanvasWidget::StepSmokeAutoRoute(
    const std::vector<localroute::Id>& conn_ids) {
  QElapsedTimer timer;
  timer.start();
  std::vector<localroute::Id> changed;
  if (mode_ == RouteMode::kSync && sync_router_) {
    changed = sync_router_->AutoRouteConnectors(conn_ids);
  } else if (mode_ == RouteMode::kAsync && async_router_) {
    changed = async_router_->AutoRouteConnectors(conn_ids);
  }
  route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  RecordRouteSample(route_time_ms_);
  UpdateRouteCacheForIds(changed);
  update();
  emit StatusChanged();
  return changed;
}

double CanvasWidget::StepSmokeUndo(bool* ok_out) {
  QElapsedTimer timer;
  timer.start();
  const bool ok = Undo();
  const double ms = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  if (ok_out) {
    *ok_out = ok;
  }
  return ms;
}

double CanvasWidget::StepSmokeRedo(bool* ok_out) {
  QElapsedTimer timer;
  timer.start();
  const bool ok = Redo();
  const double ms = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  if (ok_out) {
    *ok_out = ok;
  }
  return ms;
}

void CanvasWidget::OnAsyncTick() {
  if (mode_ != RouteMode::kAsync || !async_router_) {
    return;
  }
  const std::size_t batch_size = (drag_kind_ == DragKind::kMoveBlocks ||
                                  async_router_->PendingCount() <= 256)
                                     ? 128
                                     : 512;
  std::vector<localroute::Id> updated;
  QElapsedTimer timer;
  timer.start();
  std::size_t n = 0;
  if (!SafePollAsync(batch_size, &updated, &n)) {
    emit StatusChanged();
    return;
  }
  const double poll_ms = static_cast<double>(timer.nsecsElapsed()) / 1e6;
  if (n > 0) {
    if (drag_kind_ != DragKind::kMoveBlocks) {
      route_time_ms_ = poll_ms;
      RecordRouteSample(poll_ms);
    }
    UpdateRouteCacheForIds(updated);
    update();
    emit StatusChanged();
  } else if (async_router_->PendingCount() == 0 &&
             drag_kind_ != DragKind::kMoveBlocks) {
    async_timer_.stop();
    emit StatusChanged();
  }
}

void CanvasWidget::wheelEvent(QWheelEvent* event) {
  const QPointF cursor = event->pos();
  const QPointF world_before = ScreenToWorld(cursor);
  const double steps = static_cast<double>(event->angleDelta().y()) / 120.0;
  if (steps == 0.0) {
    return;
  }
  const double factor = std::pow(1.15, steps);
  zoom_ = std::clamp(zoom_ * factor, 0.05, 4.0);
  pan_offset_ = QPointF(cursor.x() - world_before.x() * zoom_,
                        cursor.y() - world_before.y() * zoom_);
  UpdateViewportPriority();
  update();
}

void CanvasWidget::mousePressEvent(QMouseEvent* event) {
  last_mouse_pos_ = event->pos();
  if (event->button() == Qt::MiddleButton ||
      (event->button() == Qt::LeftButton && space_pressed_)) {
    drag_kind_ = DragKind::kPan;
    setCursor(Qt::ClosedHandCursor);
    return;
  }
  if (event->button() != Qt::LeftButton) {
    return;
  }

  // 1. Check pin hit first so user can drag wires directly from any port.
  const PinHit pin_hit = HitTestPin(event->pos(), 12.0);
  if (pin_hit.valid) {
    drag_kind_ = DragKind::kWiring;
    wire_src_ = pin_hit;
    drag_start_world_ = QPointF(pin_hit.world_pos.x, pin_hit.world_pos.y);
    drag_current_world_ = drag_start_world_;
    hover_pin_ = PinHit{};
    setCursor(Qt::CrossCursor);
    update();
    return;
  }

  // 2. Check block hit for selection / drag (or Ctrl+Drag duplicate).
  const QPointF world_pos = ScreenToWorld(event->pos());
  const localroute::Id hit = HitTestBlock(world_pos);
  const auto& view = ActiveView();

  if (hit != std::numeric_limits<localroute::Id>::max()) {
    has_selected_conn_ = false;
    active_guides_.clear();
    drag_initial_rects_.clear();
    if (event->modifiers() & Qt::ControlModifier) {
      ctrl_dup_armed_ = true;
      ctrl_click_block_ = hit;
      ctrl_press_screen_ = event->pos();
      ctrl_click_was_selected_ = selected_blocks_.count(hit) != 0;
      has_pending_dup_cmd_ = false;
      drag_kind_ = DragKind::kMoveBlocks;
      drag_start_world_ = world_pos;
      if (ctrl_click_was_selected_) {
        for (localroute::Id id : selected_blocks_) {
          drag_initial_rects_.emplace(id, view.block(id).rect);
        }
      } else {
        drag_initial_rects_.emplace(hit, view.block(hit).rect);
      }
      return;
    }
    ctrl_dup_armed_ = false;
    has_pending_dup_cmd_ = false;
    if (!selected_blocks_.count(hit)) {
      selected_blocks_.clear();
      selected_blocks_.insert(hit);
    }
    drag_kind_ = DragKind::kMoveBlocks;
    drag_start_world_ = world_pos;
    highlighted_conns_.clear();
    for (localroute::Id id : selected_blocks_) {
      drag_initial_rects_.emplace(id, view.block(id).rect);
      for (localroute::Id cid : view.adjacent_conns(id)) {
        highlighted_conns_.insert(cid);
      }
    }
    UpdateViewportPriority();
    update();
    emit SelectionChanged();
    return;
  }

  // 3. Check wire segment hit: Ctrl+Drag branches a new fan-out wire from the
  //    existing signal; plain Drag moves the orthogonal segment (and any
  //    collinear same-net shared trunk).
  localroute::Id seg_cid = 0;
  std::size_t seg_idx = 0;
  bool seg_vert = false;
  if (HitTestWireSegment(event->pos(), &seg_cid, &seg_idx, &seg_vert)) {
    const auto cit = view.all_conns().find(seg_cid);
    const auto rit = view.all_routes().find(seg_cid);
    if ((event->modifiers() & Qt::ControlModifier) &&
        cit != view.all_conns().end() && rit != view.all_routes().end() &&
        seg_idx + 1 < rit->second.points.size()) {
      const localroute::Conn& base = cit->second;
      const auto& p0 = rit->second.points[seg_idx];
      const auto& p1 = rit->second.points[seg_idx + 1];
      localroute::Point branch_pt;
      if (seg_vert) {
        branch_pt = {p0.x, std::clamp(world_pos.y(), std::min(p0.y, p1.y),
                                      std::max(p0.y, p1.y))};
      } else {
        branch_pt = {std::clamp(world_pos.x(), std::min(p0.x, p1.x),
                                std::max(p0.x, p1.x)),
                     p0.y};
      }
      drag_kind_ = DragKind::kWiring;
      wire_src_ = {true, base.src, base.sp, branch_pt};
      drag_start_world_ = QPointF(branch_pt.x, branch_pt.y);
      drag_current_world_ = drag_start_world_;
      hover_pin_ = PinHit{};
      setCursor(Qt::CrossCursor);
      update();
      return;
    }

    seg_drag_before_routes_.clear();
    if (cit != view.all_conns().end()) {
      const localroute::Conn& base = cit->second;
      for (localroute::Id sib_cid : view.adjacent_conns(base.src)) {
        const auto scit = view.all_conns().find(sib_cid);
        const auto srit = view.all_routes().find(sib_cid);
        if (scit != view.all_conns().end() && srit != view.all_routes().end() &&
            scit->second.src == base.src && scit->second.sp == base.sp) {
          seg_drag_before_routes_.emplace_back(sib_cid, srit->second);
        }
      }
    }

    selected_blocks_.clear();
    highlighted_conns_.clear();
    highlighted_conns_.insert(seg_cid);
    has_selected_conn_ = true;
    selected_conn_id_ = seg_cid;
    drag_kind_ = DragKind::kMoveSegment;
    drag_seg_conn_id_ = seg_cid;
    drag_seg_index_ = seg_idx;
    drag_seg_is_vertical_ = seg_vert;
    setCursor(seg_vert ? Qt::SizeHorCursor : Qt::SizeVerCursor);
    update();
    emit SelectionChanged();
    return;
  }

  localroute::Id hit_cid = 0;
  if (!(event->modifiers() & Qt::ControlModifier) &&
      HitTestWire(event->pos(), &hit_cid)) {
    selected_blocks_.clear();
    highlighted_conns_.clear();
    highlighted_conns_.insert(hit_cid);
    has_selected_conn_ = true;
    selected_conn_id_ = hit_cid;
    update();
    emit SelectionChanged();
    return;
  }

  // 4. Empty space: rubber-band multi-select.
  if (!(event->modifiers() & Qt::ControlModifier)) {
    selected_blocks_.clear();
  }
  has_selected_conn_ = false;
  highlighted_conns_.clear();
  drag_kind_ = DragKind::kRubberBand;
  drag_start_world_ = world_pos;
  drag_current_world_ = world_pos;
  update();
  emit SelectionChanged();
}

void CanvasWidget::mouseMoveEvent(QMouseEvent* event) {
  const QPointF pos = event->pos();
  if (drag_kind_ == DragKind::kPan) {
    const QPointF delta = pos - last_mouse_pos_;
    last_mouse_pos_ = pos;
    pan_offset_ += delta;
    UpdateViewportPriority();
    update();
    return;
  }
  last_mouse_pos_ = pos;

  if (drag_kind_ == DragKind::kWiring) {
    drag_current_world_ = ScreenToWorld(pos);
    const PinHit target = HitTestPin(pos, 14.0);
    if (target.valid && (target.block_id != wire_src_.block_id ||
                         target.pin_id != wire_src_.pin_id)) {
      hover_pin_ = target;
      drag_current_world_ = QPointF(target.world_pos.x, target.world_pos.y);
    } else {
      hover_pin_ = PinHit{};
    }
    update();
    return;
  }

  if (drag_kind_ == DragKind::kRubberBand) {
    drag_current_world_ = ScreenToWorld(pos);
    update();
    return;
  }

  if (drag_kind_ == DragKind::kMoveSegment) {
    active_guides_.clear();
    const QPointF world_pos = ScreenToWorld(pos);
    double new_coord = drag_seg_is_vertical_ ? world_pos.x() : world_pos.y();

    // Smart Snap during segment drag (align straight with endpoint pins unless
    // Alt is held).
    if (!(event->modifiers() & Qt::AltModifier)) {
      const auto& view = ActiveView();
      const auto cit = view.all_conns().find(drag_seg_conn_id_);
      if (cit != view.all_conns().end()) {
        const localroute::Conn& c = cit->second;
        const localroute::Point a = PinPos(view.block(c.src), c.sp);
        const localroute::Point b = PinPos(view.block(c.dst), c.dp);
        constexpr double kSegSnapTol = 5.0;
        double best_err = kSegSnapTol + 1e-6;
        double snapped = new_coord;
        bool did_snap = false;
        for (const localroute::Point& pt : {a, b}) {
          const double ref = drag_seg_is_vertical_ ? pt.x : pt.y;
          const double err = std::abs(new_coord - ref);
          if (err <= kSegSnapTol && err < best_err) {
            best_err = err;
            snapped = ref;
            did_snap = true;
          }
        }
        if (did_snap) {
          new_coord = snapped;
          double s0 =
              drag_seg_is_vertical_ ? std::min(a.y, b.y) : std::min(a.x, b.x);
          double s1 =
              drag_seg_is_vertical_ ? std::max(a.y, b.y) : std::max(a.x, b.x);
          const auto rit = view.all_routes().find(drag_seg_conn_id_);
          if (rit != view.all_routes().end() &&
              drag_seg_index_ + 1 < rit->second.points.size()) {
            const auto& p0 = rit->second.points[drag_seg_index_];
            const auto& p1 = rit->second.points[drag_seg_index_ + 1];
            if (drag_seg_is_vertical_) {
              s0 = std::min({s0, p0.y, p1.y});
              s1 = std::max({s1, p0.y, p1.y});
            } else {
              s0 = std::min({s0, p0.x, p1.x});
              s1 = std::max({s1, p0.x, p1.x});
            }
          }
          active_guides_.push_back(
              {drag_seg_is_vertical_, new_coord, s0 - 24.0, s1 + 24.0});
        }
      }
    }

    QElapsedTimer timer;
    timer.start();
    bool ok = false;
    std::vector<localroute::Id> changed;
    if (mode_ == RouteMode::kSync && sync_router_) {
      ok = sync_router_->MoveRouteSegment(drag_seg_conn_id_, drag_seg_index_,
                                          new_coord, &changed);
    } else if (mode_ == RouteMode::kAsync && async_router_) {
      ok = async_router_->MoveRouteSegment(drag_seg_conn_id_, drag_seg_index_,
                                           new_coord, &changed);
    }
    route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
    RecordRouteSample(route_time_ms_);
    if (ok) {
      const auto& view = ActiveView();
      const auto rit = view.all_routes().find(drag_seg_conn_id_);
      if (rit != view.all_routes().end() && rit->second.points.size() >= 2) {
        const auto& pts = rit->second.points;
        double best_err = std::numeric_limits<double>::max();
        bool best_mid = false;
        std::size_t best_j = 0;
        for (std::size_t j = 0; j + 1 < pts.size(); ++j) {
          const bool is_vert = std::abs(pts[j].x - pts[j + 1].x) <= 1e-6 &&
                               std::abs(pts[j].y - pts[j + 1].y) > 1e-6;
          const bool is_horiz = std::abs(pts[j].y - pts[j + 1].y) <= 1e-6 &&
                                std::abs(pts[j].x - pts[j + 1].x) > 1e-6;
          if ((drag_seg_is_vertical_ && !is_vert) ||
              (!drag_seg_is_vertical_ && !is_horiz)) {
            continue;
          }
          const double coord = drag_seg_is_vertical_ ? pts[j].x : pts[j].y;
          const double err = std::abs(coord - new_coord);
          const bool is_mid = (pts.size() >= 4 && j >= 1 && j + 2 < pts.size());
          if (err < best_err - 1e-6 ||
              (std::abs(err - best_err) <= 1e-6 && is_mid && !best_mid)) {
            best_err = err;
            best_mid = is_mid;
            best_j = j;
          }
        }
        if (best_err < std::numeric_limits<double>::max()) {
          drag_seg_index_ = best_j;
        }
      }
      UpdateRouteCacheForIds(
          changed.empty() ? std::vector<localroute::Id>{drag_seg_conn_id_}
                          : changed);
      update();
      emit StatusChanged();
    }
    return;
  }

  if (drag_kind_ == DragKind::kMoveBlocks && !drag_initial_rects_.empty()) {
    if (ctrl_dup_armed_) {
      const QPointF delta_px = pos - ctrl_press_screen_;
      if (delta_px.x() * delta_px.x() + delta_px.y() * delta_px.y() < 25.0) {
        return;
      }
      ctrl_dup_armed_ = false;
      const auto& view = ActiveView();
      std::unordered_map<localroute::Id, localroute::Id> old_to_new;
      std::unordered_map<localroute::Id, localroute::Rect> new_initial;
      EditCommand dup_cmd;
      dup_cmd.kind = EditCommand::Kind::kAddItems;

      selected_blocks_.clear();
      highlighted_conns_.clear();
      for (const auto& kv : drag_initial_rects_) {
        if (!view.all_blocks().count(kv.first)) {
          continue;
        }
        localroute::Block copy = view.block(kv.first);
        copy.id = next_block_id_++;
        old_to_new.emplace(kv.first, copy.id);
        new_initial.emplace(copy.id, copy.rect);
        selected_blocks_.insert(copy.id);
        dup_cmd.blocks.push_back(copy);
        AddBlockIncremental(std::move(copy));
      }
      for (const auto& kv : view.all_conns()) {
        const localroute::Conn& c = kv.second;
        const auto sit = old_to_new.find(c.src);
        const auto dit = old_to_new.find(c.dst);
        if (sit != old_to_new.end() && dit != old_to_new.end()) {
          const localroute::Id new_cid = next_conn_id_++;
          const localroute::Conn new_conn{new_cid, sit->second, c.sp,
                                          dit->second, c.dp};
          dup_cmd.conns.push_back(new_conn);
          highlighted_conns_.insert(new_cid);
          AddConnectorIncremental(new_conn);
        }
      }
      drag_initial_rects_ = std::move(new_initial);
      pending_dup_cmd_ = std::move(dup_cmd);
      has_pending_dup_cmd_ = true;
      current_block_count_ =
          static_cast<unsigned>(ActiveView().all_blocks().size());
      current_conn_count_ =
          static_cast<unsigned>(ActiveView().all_conns().size());
      emit SelectionChanged();
    }

    const QPointF world_pos = ScreenToWorld(pos);
    double dx = world_pos.x() - drag_start_world_.x();
    double dy = world_pos.y() - drag_start_world_.y();
    active_guides_.clear();

    // Smart Guides: snap single-block drag to aligned peer port coordinates
    // unless Alt is held.
    if (drag_initial_rects_.size() == 1 &&
        !(event->modifiers() & Qt::AltModifier)) {
      const auto& view = ActiveView();
      const localroute::Id bid = drag_initial_rects_.begin()->first;
      const localroute::Rect init_r = drag_initial_rects_.begin()->second;
      const localroute::Rect raw_r{init_r.l + dx, init_r.t + dy, init_r.r + dx,
                                   init_r.b + dy};
      const localroute::Block& blk = view.block(bid);
      const auto find_pin = [](const localroute::Block& b,
                               localroute::Id pid) -> const localroute::Pin* {
        for (const auto& p : b.pins) {
          if (p.id == pid) {
            return &p;
          }
        }
        return nullptr;
      };
      const auto pin_at_rect =
          [](localroute::Rect r,
             const localroute::Pin& p) -> localroute::Point {
        return {p.x == 1.0 ? r.r : r.l + p.x * (r.r - r.l),
                p.y == 1.0 ? r.b : r.t + p.y * (r.b - r.t)};
      };

      constexpr double kSnapTol = 5.0;
      double best_dy_err = kSnapTol + 1e-6;
      double snap_dy = 0.0;
      bool has_snap_y = false;
      double best_dx_err = kSnapTol + 1e-6;
      double snap_dx = 0.0;
      bool has_snap_x = false;

      for (localroute::Id cid : view.adjacent_conns(bid)) {
        const auto cit = view.all_conns().find(cid);
        if (cit == view.all_conns().end() ||
            cit->second.src == cit->second.dst) {
          continue;
        }
        const localroute::Conn& c = cit->second;
        const bool is_src = (c.src == bid);
        const localroute::Id my_pid = is_src ? c.sp : c.dp;
        const localroute::Id peer_bid = is_src ? c.dst : c.src;
        const localroute::Id peer_pid = is_src ? c.dp : c.sp;
        const localroute::Pin* my_pin = find_pin(blk, my_pid);
        const localroute::Block& peer_blk = view.block(peer_bid);
        const localroute::Pin* peer_pin = find_pin(peer_blk, peer_pid);
        if (!my_pin || !peer_pin) {
          continue;
        }
        const localroute::Point my_pt = pin_at_rect(raw_r, *my_pin);
        const localroute::Point peer_pt = pin_at_rect(peer_blk.rect, *peer_pin);
        if ((my_pin->dirs & (localroute::kLeft | localroute::kRight)) &&
            (peer_pin->dirs & (localroute::kLeft | localroute::kRight))) {
          const double err = std::abs(my_pt.y - peer_pt.y);
          if (err <= kSnapTol && err < best_dy_err) {
            best_dy_err = err;
            snap_dy = peer_pt.y - my_pt.y;
            has_snap_y = true;
          }
        }
        if ((my_pin->dirs & (localroute::kUp | localroute::kDown)) &&
            (peer_pin->dirs & (localroute::kUp | localroute::kDown))) {
          const double err = std::abs(my_pt.x - peer_pt.x);
          if (err <= kSnapTol && err < best_dx_err) {
            best_dx_err = err;
            snap_dx = peer_pt.x - my_pt.x;
            has_snap_x = true;
          }
        }
      }
      if (has_snap_y) {
        dy += snap_dy;
      }
      if (has_snap_x) {
        dx += snap_dx;
      }
      if (has_snap_x || has_snap_y) {
        const localroute::Rect snapped_r{init_r.l + dx, init_r.t + dy,
                                         init_r.r + dx, init_r.b + dy};
        for (localroute::Id cid : view.adjacent_conns(bid)) {
          const auto cit = view.all_conns().find(cid);
          if (cit == view.all_conns().end() ||
              cit->second.src == cit->second.dst) {
            continue;
          }
          const localroute::Conn& c = cit->second;
          const bool is_src = (c.src == bid);
          const localroute::Pin* my_pin = find_pin(blk, is_src ? c.sp : c.dp);
          const localroute::Block& peer_blk =
              view.block(is_src ? c.dst : c.src);
          const localroute::Pin* peer_pin =
              find_pin(peer_blk, is_src ? c.dp : c.sp);
          if (!my_pin || !peer_pin) {
            continue;
          }
          const localroute::Point my_pt = pin_at_rect(snapped_r, *my_pin);
          const localroute::Point peer_pt =
              pin_at_rect(peer_blk.rect, *peer_pin);
          if (has_snap_y &&
              (my_pin->dirs & (localroute::kLeft | localroute::kRight)) &&
              (peer_pin->dirs & (localroute::kLeft | localroute::kRight)) &&
              std::abs(my_pt.y - peer_pt.y) <= 1e-4) {
            active_guides_.push_back({false, peer_pt.y,
                                      std::min(my_pt.x, peer_pt.x) - 36.0,
                                      std::max(my_pt.x, peer_pt.x) + 36.0});
          }
          if (has_snap_x &&
              (my_pin->dirs & (localroute::kUp | localroute::kDown)) &&
              (peer_pin->dirs & (localroute::kUp | localroute::kDown)) &&
              std::abs(my_pt.x - peer_pt.x) <= 1e-4) {
            active_guides_.push_back({true, peer_pt.x,
                                      std::min(my_pt.y, peer_pt.y) - 36.0,
                                      std::max(my_pt.y, peer_pt.y) + 36.0});
          }
        }
      }
    }

    std::vector<std::pair<localroute::Id, localroute::Rect>> moves;
    std::vector<localroute::Id> moved_ids;
    moves.reserve(drag_initial_rects_.size());
    moved_ids.reserve(drag_initial_rects_.size());
    for (const auto& kv : drag_initial_rects_) {
      const auto& r = kv.second;
      moves.push_back({kv.first, {r.l + dx, r.t + dy, r.r + dx, r.b + dy}});
      moved_ids.push_back(kv.first);
    }

    QElapsedTimer timer;
    timer.start();
    if (mode_ == RouteMode::kSync && sync_router_) {
      sync_router_->MoveBlocksBatch(moves);
      const auto changed = sync_router_->RerouteAffected(moved_ids);
      route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
      UpdateRouteCacheForIds(changed);
      if (third_party_reroute_) {
        highlighted_conns_.clear();
        for (localroute::Id cid : changed) {
          highlighted_conns_.insert(cid);
        }
      }
    } else if (mode_ == RouteMode::kAsync && async_router_) {
      async_router_->Move(moves);
      std::vector<localroute::Id> updated;
      const bool poll_ok = SafePollAsync(128, &updated);
      route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
      for (localroute::Id id : moved_ids) {
        UpdateRouteBoundsFromBlock(id);
      }
      if (!updated.empty()) {
        UpdateRouteCacheForIds(updated);
      }
      if (poll_ok && !async_timer_.isActive()) {
        async_timer_.start();
      }
    }
    RecordRouteSample(route_time_ms_);
    update();
    emit StatusChanged();
    return;
  }

  // Idle hover highlight on pins and wire segments
  if (drag_kind_ == DragKind::kNone && !space_pressed_) {
    const PinHit p = HitTestPin(pos, 12.0);
    const bool changed = (p.valid != hover_pin_.valid) ||
                         (p.valid && (p.block_id != hover_pin_.block_id ||
                                      p.pin_id != hover_pin_.pin_id));
    if (changed) {
      hover_pin_ = p;
      update();
    }
    if (p.valid) {
      setCursor(Qt::CrossCursor);
    } else if (zoom_ >= 0.25 &&
               HitTestBlock(ScreenToWorld(pos)) ==
                   std::numeric_limits<localroute::Id>::max()) {
      localroute::Id seg_cid = 0;
      std::size_t seg_idx = 0;
      bool seg_vert = false;
      if (HitTestWireSegment(pos, &seg_cid, &seg_idx, &seg_vert)) {
        if (event->modifiers() & Qt::ControlModifier) {
          setCursor(Qt::CrossCursor);
        } else {
          setCursor(seg_vert ? Qt::SizeHorCursor : Qt::SizeVerCursor);
        }
      } else {
        setCursor(Qt::ArrowCursor);
      }
    } else {
      setCursor(Qt::ArrowCursor);
    }
  }
}

void CanvasWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (drag_kind_ == DragKind::kPan) {
    drag_kind_ = DragKind::kNone;
    setCursor(space_pressed_ ? Qt::OpenHandCursor : Qt::ArrowCursor);
    return;
  }
  if (event->button() != Qt::LeftButton) {
    return;
  }

  if (drag_kind_ == DragKind::kWiring) {
    drag_kind_ = DragKind::kNone;
    setCursor(Qt::ArrowCursor);
    const PinHit target = hover_pin_;
    hover_pin_ = PinHit{};
    if (wire_src_.valid && target.valid &&
        (target.block_id != wire_src_.block_id ||
         target.pin_id != wire_src_.pin_id)) {
      AddWire(wire_src_.block_id, wire_src_.pin_id, target.block_id,
              target.pin_id);
    } else {
      update();
    }
    wire_src_ = PinHit{};
    return;
  }

  if (drag_kind_ == DragKind::kRubberBand) {
    const localroute::Rect sel_box{
        std::min(drag_start_world_.x(), drag_current_world_.x()),
        std::min(drag_start_world_.y(), drag_current_world_.y()),
        std::max(drag_start_world_.x(), drag_current_world_.x()),
        std::max(drag_start_world_.y(), drag_current_world_.y())};
    const auto& view = ActiveView();
    for (localroute::Id id : view.QueryBlocks(sel_box)) {
      if (view.block(id).rect.Intersects(sel_box)) {
        selected_blocks_.insert(id);
      }
    }
    highlighted_conns_.clear();
    for (localroute::Id id : selected_blocks_) {
      for (localroute::Id cid : view.adjacent_conns(id)) {
        highlighted_conns_.insert(cid);
      }
    }
    drag_kind_ = DragKind::kNone;
    UpdateViewportPriority();
    update();
    emit SelectionChanged();
    return;
  }

  if (drag_kind_ == DragKind::kMoveSegment) {
    drag_kind_ = DragKind::kNone;
    active_guides_.clear();
    setCursor(Qt::ArrowCursor);

    EditCommand cmd;
    cmd.kind = EditCommand::Kind::kModifyRoutes;
    const auto& all_routes = ActiveView().all_routes();
    for (const auto& kv : seg_drag_before_routes_) {
      const auto rit = all_routes.find(kv.first);
      if (rit != all_routes.end() &&
          (!(rit->second.points == kv.second.points) ||
           rit->second.status != kv.second.status ||
           rit->second.user_modified != kv.second.user_modified)) {
        cmd.before_routes.push_back(kv);
        cmd.after_routes.emplace_back(kv.first, rit->second);
      }
    }
    seg_drag_before_routes_.clear();
    if (!cmd.after_routes.empty()) {
      PushUndoCommand(std::move(cmd));
    }

    update();
    emit StatusChanged();
    return;
  }

  if (drag_kind_ == DragKind::kMoveBlocks) {
    drag_kind_ = DragKind::kNone;
    active_guides_.clear();
    if (ctrl_dup_armed_) {
      ctrl_dup_armed_ = false;
      drag_initial_rects_.clear();
      if (ctrl_click_was_selected_) {
        selected_blocks_.erase(ctrl_click_block_);
      } else {
        selected_blocks_.insert(ctrl_click_block_);
      }
      highlighted_conns_.clear();
      const auto& view = ActiveView();
      for (localroute::Id id : selected_blocks_) {
        for (localroute::Id cid : view.adjacent_conns(id)) {
          highlighted_conns_.insert(cid);
        }
      }
      UpdateViewportPriority();
      update();
      emit SelectionChanged();
      return;
    }

    if (mode_ == RouteMode::kSync && sync_router_) {
      std::vector<localroute::Id> moved_ids;
      for (const auto& kv : drag_initial_rects_) {
        moved_ids.push_back(kv.first);
      }
      QElapsedTimer timer;
      timer.start();
      const auto changed = sync_router_->RerouteAffected(moved_ids, true);
      route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
      RecordRouteSample(route_time_ms_);
      UpdateRouteCacheForIds(changed);
    } else if (mode_ == RouteMode::kAsync && async_router_) {
      async_router_->Settle();
      if (!async_timer_.isActive()) {
        async_timer_.start();
      }
    }

    const auto& view = ActiveView();
    if (has_pending_dup_cmd_) {
      has_pending_dup_cmd_ = false;
      for (auto& blk : pending_dup_cmd_.blocks) {
        if (view.all_blocks().count(blk.id)) {
          blk.rect = view.block(blk.id).rect;
        }
      }
      for (const auto& c : pending_dup_cmd_.conns) {
        if (const auto rit = view.all_routes().find(c.id);
            rit != view.all_routes().end()) {
          pending_dup_cmd_.after_routes.emplace_back(c.id, rit->second);
        }
      }
      PushUndoCommand(std::move(pending_dup_cmd_));
    } else {
      EditCommand move_cmd;
      move_cmd.kind = EditCommand::Kind::kMoveBlocks;
      bool any_moved = false;
      for (const auto& kv : drag_initial_rects_) {
        if (!view.all_blocks().count(kv.first)) {
          continue;
        }
        const localroute::Rect after_r = view.block(kv.first).rect;
        if (std::abs(after_r.l - kv.second.l) > 1e-6 ||
            std::abs(after_r.t - kv.second.t) > 1e-6 ||
            std::abs(after_r.r - kv.second.r) > 1e-6 ||
            std::abs(after_r.b - kv.second.b) > 1e-6) {
          any_moved = true;
        }
        move_cmd.before_rects.push_back(kv);
        move_cmd.after_rects.emplace_back(kv.first, after_r);
      }
      if (any_moved) {
        PushUndoCommand(std::move(move_cmd));
      }
    }

    drag_initial_rects_.clear();
    RefreshWorldBounds();
    update();
    emit StatusChanged();
  }
}

void CanvasWidget::keyPressEvent(QKeyEvent* event) {
  if ((event->modifiers() & Qt::ControlModifier) && event->key() == Qt::Key_Z) {
    if (event->modifiers() & Qt::ShiftModifier) {
      (void)Redo();
    } else {
      (void)Undo();
    }
    return;
  }
  if ((event->modifiers() & Qt::ControlModifier) && event->key() == Qt::Key_Y) {
    (void)Redo();
    return;
  }
  if (event->key() == Qt::Key_Space && !event->isAutoRepeat()) {
    space_pressed_ = true;
    if (drag_kind_ == DragKind::kNone) {
      setCursor(Qt::OpenHandCursor);
    }
  } else if (event->key() == Qt::Key_Delete ||
             event->key() == Qt::Key_Backspace) {
    DeleteSelection();
  } else if (event->key() == Qt::Key_R) {
    ResetLocalView();
  } else if (event->key() == Qt::Key_F) {
    FitAll();
  } else {
    QWidget::keyPressEvent(event);
  }
}

void CanvasWidget::keyReleaseEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Space && !event->isAutoRepeat()) {
    space_pressed_ = false;
    if (drag_kind_ == DragKind::kNone) {
      setCursor(Qt::ArrowCursor);
    }
  } else {
    QWidget::keyReleaseEvent(event);
  }
}

void CanvasWidget::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  UpdateViewportPriority();
}

void CanvasWidget::dragEnterEvent(QDragEnterEvent* event) {
  if (event->mimeData()->hasFormat(kBlockTemplateMime)) {
    BlockTemplate tmpl;
    if (DecodeBlockTemplate(event->mimeData()->data(kBlockTemplateMime),
                            &tmpl)) {
      drop_ghost_tmpl_ = tmpl;
      drop_ghost_world_ = ScreenToWorld(event->pos());
      has_drop_ghost_ = true;
      event->acceptProposedAction();
      update();
      return;
    }
  }
  event->ignore();
}

void CanvasWidget::dragMoveEvent(QDragMoveEvent* event) {
  if (event->mimeData()->hasFormat(kBlockTemplateMime)) {
    drop_ghost_world_ = ScreenToWorld(event->pos());
    has_drop_ghost_ = true;
    event->acceptProposedAction();
    update();
    return;
  }
  event->ignore();
}

void CanvasWidget::dragLeaveEvent(QDragLeaveEvent* /*event*/) {
  has_drop_ghost_ = false;
  update();
}

void CanvasWidget::dropEvent(QDropEvent* event) {
  has_drop_ghost_ = false;
  if (event->mimeData()->hasFormat(kBlockTemplateMime)) {
    BlockTemplate tmpl;
    if (DecodeBlockTemplate(event->mimeData()->data(kBlockTemplateMime),
                            &tmpl)) {
      AddBlockFromTemplate(tmpl, ScreenToWorld(event->pos()));
      setFocus();
      event->acceptProposedAction();
      return;
    }
  }
  event->ignore();
  update();
}

void CanvasWidget::contextMenuEvent(QContextMenuEvent* event) {
  const QPointF world_pos = ScreenToWorld(event->pos());
  const localroute::Id hit_blk = HitTestBlock(world_pos);
  if (hit_blk != std::numeric_limits<localroute::Id>::max()) {
    selected_blocks_.clear();
    selected_blocks_.insert(hit_blk);
    has_selected_conn_ = false;
    emit SelectionChanged();
    update();

    localroute::Id bid = 0;
    int l = 0;
    int r = 0;
    int t = 0;
    int b = 0;
    GetSelectedBlockPorts(&bid, &l, &r, &t, &b);

    QMenu menu(this);
    QAction* add_in = menu.addAction("+1 左输入端口");
    QAction* sub_in = menu.addAction("-1 左输入端口");
    sub_in->setEnabled(l > 0);
    QAction* add_out = menu.addAction("+1 右输出端口");
    QAction* sub_out = menu.addAction("-1 右输出端口");
    sub_out->setEnabled(r > 0);
    menu.addSeparator();
    QAction* make_obs = menu.addAction("转为 0 端口障碍物");
    QAction* clear_wires = menu.addAction("清除该模块连线");
    QAction* del_blk = menu.addAction("删除模块 (Del)");

    QAction* chosen = menu.exec(event->globalPos());
    if (chosen == add_in) {
      SetBlockPorts(bid, l + 1, r, t, b);
    } else if (chosen == sub_in) {
      SetBlockPorts(bid, l - 1, r, t, b);
    } else if (chosen == add_out) {
      SetBlockPorts(bid, l, r + 1, t, b);
    } else if (chosen == sub_out) {
      SetBlockPorts(bid, l, r - 1, t, b);
    } else if (chosen == make_obs) {
      SetBlockPorts(bid, 0, 0, 0, 0);
    } else if (chosen == clear_wires) {
      ClearBlockWires(bid);
    } else if (chosen == del_blk) {
      DeleteSelection();
    }
    return;
  }

  localroute::Id hit_cid = 0;
  if (HitTestWire(event->pos(), &hit_cid)) {
    selected_blocks_.clear();
    highlighted_conns_.clear();
    highlighted_conns_.insert(hit_cid);
    has_selected_conn_ = true;
    selected_conn_id_ = hit_cid;
    emit SelectionChanged();
    update();

    QMenu menu(this);
    QAction* del_wire = menu.addAction("删除连线 (Del)");
    if (menu.exec(event->globalPos()) == del_wire) {
      DeleteSelection();
    }
  }
}

void CanvasWidget::paintEvent(QPaintEvent* /*event*/) {
  QElapsedTimer paint_timer;
  paint_timer.start();

  const qint64 now_ms = fps_clock_.elapsed();
  frame_timestamps_ms_.push_back(now_ms);
  while (frame_timestamps_ms_.size() > 1 &&
         now_ms - frame_timestamps_ms_.front() > 1000) {
    frame_timestamps_ms_.pop_front();
  }
  if (frame_timestamps_ms_.size() >= 2) {
    const qint64 span_ms = now_ms - frame_timestamps_ms_.front();
    current_fps_ = span_ms > 0
                       ? static_cast<double>(frame_timestamps_ms_.size() - 1) *
                             1000.0 / static_cast<double>(span_ms)
                       : 0.0;
  }

  QPainter painter(this);
  painter.fillRect(rect(), QColor(246, 248, 251));

  const bool high_detail = zoom_ >= 0.35;
  const bool mid_detail = zoom_ >= 0.15 && zoom_ < 0.35;
  const bool low_detail = zoom_ < 0.15;

  painter.save();
  painter.translate(pan_offset_);
  painter.scale(zoom_, zoom_);

  const localroute::Rect vp = WorldViewportRect();
  const auto& view = ActiveView();
  const auto& routes = view.all_routes();

  painter.setRenderHint(QPainter::Antialiasing, false);
  QPen valid_pen(QColor(30, 78, 140), low_detail ? 1.0 : 1.35);
  valid_pen.setCosmetic(true);
  QPen pending_pen(
      low_detail ? QColor(115, 134, 158, 140) : QColor(115, 134, 158), 1.2,
      low_detail ? Qt::SolidLine : Qt::DashLine);
  pending_pen.setCosmetic(true);
  QPen display_pen =
      highlight_fallback_ ? QPen(QColor(230, 126, 34), 1.5) : valid_pen;
  display_pen.setCosmetic(true);
  QPen warn_pen(QColor(220, 38, 38), 1.8, Qt::DashLine);
  warn_pen.setCosmetic(true);
  QPen hi_valid_pen(QColor(0, 120, 215), 2.2);
  hi_valid_pen.setCosmetic(true);
  QPen hi_pending_pen(QColor(0, 120, 215), 1.8, Qt::DashLine);
  hi_pending_pen.setCosmetic(true);
  QPen hi_display_pen =
      highlight_fallback_ ? QPen(QColor(230, 126, 34), 2.2) : hi_valid_pen;
  hi_display_pen.setCosmetic(true);
  QPen hi_warn_pen(QColor(220, 38, 38), 2.4, Qt::DashLine);
  hi_warn_pen.setCosmetic(true);

  const bool draw_solder = zoom_ >= 0.25;
  const bool draw_hops = zoom_ >= 0.4 && show_line_hops_;
  const bool draw_arrows = zoom_ >= 0.45;

  struct VisHSeg {
    double x0 = 0.0;
    double x1 = 0.0;
    double y = 0.0;
    localroute::Id src = 0;
    localroute::Id sp = 0;
    int bucket = 0;  // 0: valid, 1: display, 2: hi_valid, 3: hi_display,
                     // 4: warn, 5: hi_warn
  };
  struct VisVSeg {
    double x = 0.0;
    double y0 = 0.0;
    double y1 = 0.0;
    localroute::Id src = 0;
    localroute::Id sp = 0;
  };
  struct ArrowHead {
    double x = 0.0;
    double y = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    int bucket = 0;
  };

  QVector<QLineF> valid_segs;
  QVector<QLineF> pending_segs;
  QVector<QLineF> display_segs;
  QVector<QLineF> warn_segs;
  QVector<QLineF> hi_valid_segs;
  QVector<QLineF> hi_pending_segs;
  QVector<QLineF> hi_display_segs;
  QVector<QLineF> hi_warn_segs;
  valid_segs.reserve(2048);
  pending_segs.reserve(1024);
  display_segs.reserve(128);
  std::vector<localroute::Id> visible_conn_ids;
  if (draw_solder) {
    visible_conn_ids.reserve(512);
  }
  std::vector<VisHSeg> vis_h;
  std::vector<VisVSeg> vis_v;
  std::array<std::vector<QRectF>, 6> hop_arcs;
  if (draw_hops) {
    vis_h.reserve(1024);
    vis_v.reserve(1024);
  }
  std::vector<ArrowHead> arrows;
  if (draw_arrows) {
    arrows.reserve(512);
  }
  std::vector<localroute::Id> top_highlight;
  top_highlight.reserve(highlighted_conns_.size());

  auto record_arrow = [&](int bucket,
                          const std::vector<localroute::Point>& pts) {
    if (!draw_arrows || pts.size() < 2) {
      return;
    }
    const localroute::Point tip = pts.back();
    if (tip.x < vp.l - 12.0 || tip.x > vp.r + 12.0 || tip.y < vp.t - 12.0 ||
        tip.y > vp.b + 12.0) {
      return;
    }
    for (std::size_t k = pts.size() - 1; k > 0; --k) {
      const double dx = tip.x - pts[k - 1].x;
      const double dy = tip.y - pts[k - 1].y;
      const double len = std::hypot(dx, dy);
      if (len > 1e-6) {
        arrows.push_back({tip.x, tip.y, dx / len, dy / len, bucket});
        return;
      }
    }
  };

  auto append_clipped = [&](QVector<QLineF>* dst, localroute::Point p0,
                            localroute::Point p1) {
    if (p0.x == p1.x) {
      if (p0.x < vp.l || p0.x > vp.r) {
        return;
      }
      const double y0 = std::clamp(std::min(p0.y, p1.y), vp.t, vp.b);
      const double y1 = std::clamp(std::max(p0.y, p1.y), vp.t, vp.b);
      if (y0 < y1) {
        dst->append(QLineF(p0.x, y0, p0.x, y1));
      }
    } else if (p0.y == p1.y) {
      if (p0.y < vp.t || p0.y > vp.b) {
        return;
      }
      const double x0 = std::clamp(std::min(p0.x, p1.x), vp.l, vp.r);
      const double x1 = std::clamp(std::max(p0.x, p1.x), vp.l, vp.r);
      if (x0 < x1) {
        dst->append(QLineF(x0, p0.y, x1, p0.y));
      }
    } else if (localroute::Box(p0, p1).Intersects(vp)) {
      dst->append(QLineF(p0.x, p0.y, p1.x, p1.y));
    }
  };

  auto bucket_lines = [&](int bucket) -> QVector<QLineF>* {
    switch (bucket) {
      case 0:
        return &valid_segs;
      case 1:
        return &display_segs;
      case 2:
        return &hi_valid_segs;
      case 3:
        return &hi_display_segs;
      case 4:
        return &warn_segs;
      default:
        return &hi_warn_segs;
    }
  };

  auto append_hop_candidate = [&](int bucket, const localroute::Conn& conn,
                                  localroute::Point p0, localroute::Point p1) {
    QVector<QLineF>* dst = bucket_lines(bucket);
    if (p0.x == p1.x) {
      if (p0.x < vp.l || p0.x > vp.r) {
        return;
      }
      const double y0 = std::clamp(std::min(p0.y, p1.y), vp.t, vp.b);
      const double y1 = std::clamp(std::max(p0.y, p1.y), vp.t, vp.b);
      if (y0 < y1) {
        dst->append(QLineF(p0.x, y0, p0.x, y1));
        vis_v.push_back({p0.x, y0, y1, conn.src, conn.sp});
      }
    } else if (p0.y == p1.y) {
      if (p0.y < vp.t || p0.y > vp.b) {
        return;
      }
      const double x0 = std::clamp(std::min(p0.x, p1.x), vp.l, vp.r);
      const double x1 = std::clamp(std::max(p0.x, p1.x), vp.l, vp.r);
      if (x0 < x1) {
        vis_h.push_back({x0, x1, p0.y, conn.src, conn.sp, bucket});
      }
    } else if (localroute::Box(p0, p1).Intersects(vp)) {
      dst->append(QLineF(p0.x, p0.y, p1.x, p1.y));
    }
  };

  const auto& all_blocks = view.all_blocks();
  const auto& all_conns = view.all_conns();
  std::size_t drawn_conns = 0;
  for (const auto& entry : route_cache_) {
    if (highlighted_conns_.count(entry.id)) {
      top_highlight.push_back(entry.id);
      if (draw_solder && entry.bounds.Intersects(vp)) {
        visible_conn_ids.push_back(entry.id);
      }
      continue;
    }
    if (!entry.bounds.Intersects(vp)) {
      continue;
    }
    ++drawn_conns;
    if (draw_solder) {
      visible_conn_ids.push_back(entry.id);
    }
    if (entry.status == localroute::Status::kPending) {
      const auto& c = all_conns.at(entry.id);
      const localroute::Point a = PinPos(all_blocks.at(c.src), c.sp);
      const localroute::Point b = PinPos(all_blocks.at(c.dst), c.dp);
      if (a.x == b.x || a.y == b.y) {
        append_clipped(&pending_segs, a, b);
      } else {
        const localroute::Point corner{b.x, a.y};
        append_clipped(&pending_segs, a, corner);
        append_clipped(&pending_segs, corner, b);
      }
    } else {
      const auto rit = routes.find(entry.id);
      if (rit == routes.end() || rit->second.points.size() < 2) {
        continue;
      }
      const auto& pts = rit->second.points;
      const int bucket =
          (rit->second.status == localroute::Status::kValid)
              ? 0
              : ((rit->second.user_modified &&
                  rit->second.status == localroute::Status::kDisplayOnly)
                     ? 4
                     : 1);
      record_arrow(bucket, pts);
      if (draw_hops) {
        const auto& c = all_conns.at(entry.id);
        for (std::size_t j = 1; j < pts.size(); ++j) {
          append_hop_candidate(bucket, c, pts[j - 1], pts[j]);
        }
      } else {
        QVector<QLineF>* dst = bucket_lines(bucket);
        for (std::size_t j = 1; j < pts.size(); ++j) {
          append_clipped(dst, pts[j - 1], pts[j]);
        }
      }
    }
  }

  for (localroute::Id cid : top_highlight) {
    ++drawn_conns;
    const auto rit = routes.find(cid);
    if (rit == routes.end() ||
        rit->second.status == localroute::Status::kPending ||
        rit->second.points.empty()) {
      const auto& c = all_conns.at(cid);
      const localroute::Point a = PinPos(all_blocks.at(c.src), c.sp);
      const localroute::Point b = PinPos(all_blocks.at(c.dst), c.dp);
      if (a.x == b.x || a.y == b.y) {
        append_clipped(&hi_pending_segs, a, b);
      } else {
        const localroute::Point corner{b.x, a.y};
        append_clipped(&hi_pending_segs, a, corner);
        append_clipped(&hi_pending_segs, corner, b);
      }
    } else {
      const auto& pts = rit->second.points;
      const int bucket =
          (rit->second.status == localroute::Status::kValid)
              ? 2
              : ((rit->second.user_modified &&
                  rit->second.status == localroute::Status::kDisplayOnly)
                     ? 5
                     : 3);
      record_arrow(bucket, pts);
      if (draw_hops) {
        const auto& c = all_conns.at(cid);
        for (std::size_t j = 1; j < pts.size(); ++j) {
          append_hop_candidate(bucket, c, pts[j - 1], pts[j]);
        }
      } else {
        QVector<QLineF>* dst = bucket_lines(bucket);
        for (std::size_t j = 1; j < pts.size(); ++j) {
          append_clipped(dst, pts[j - 1], pts[j]);
        }
      }
    }
  }

  if (draw_hops && !vis_h.empty()) {
    const double hop_r = std::clamp(4.5 / std::max(zoom_, 0.4), 3.5, 7.0);
    std::sort(vis_v.begin(), vis_v.end(),
              [](const VisVSeg& a, const VisVSeg& b) { return a.x < b.x; });
    std::vector<double> crossings;
    for (const VisHSeg& h : vis_h) {
      QVector<QLineF>* dst = bucket_lines(h.bucket);
      if (h.x1 - h.x0 <= 2.0 * hop_r || vis_v.empty()) {
        dst->append(QLineF(h.x0, h.y, h.x1, h.y));
        continue;
      }
      crossings.clear();
      const auto it0 = std::upper_bound(
          vis_v.begin(), vis_v.end(), h.x0 + hop_r,
          [](double val, const VisVSeg& v) { return val < v.x; });
      for (auto it = it0; it != vis_v.end() && it->x < h.x1 - hop_r; ++it) {
        if (it->src == h.src && it->sp == h.sp) {
          continue;
        }
        if (it->y0 + hop_r < h.y && h.y < it->y1 - hop_r) {
          crossings.push_back(it->x);
        }
      }
      if (crossings.empty()) {
        dst->append(QLineF(h.x0, h.y, h.x1, h.y));
        continue;
      }
      std::sort(crossings.begin(), crossings.end());
      std::size_t write = 0;
      for (double cx : crossings) {
        if (write == 0 || cx - crossings[write - 1] > 1e-3) {
          crossings[write++] = cx;
        }
      }
      crossings.resize(write);

      double cur_x = h.x0;
      for (std::size_t i = 0; i < crossings.size();) {
        std::size_t j = i;
        while (j + 1 < crossings.size() &&
               crossings[j + 1] - crossings[j] < 3.0) {
          ++j;
        }
        const double ix_l = crossings[i];
        const double ix_r = crossings[j];
        double r_l = std::min(hop_r, ix_l - cur_x);
        double r_r = std::min(hop_r, h.x1 - ix_r);
        if (i > 0) {
          r_l = std::min(r_l, 0.45 * (ix_l - crossings[i - 1]));
        }
        if (j + 1 < crossings.size()) {
          r_r = std::min(r_r, 0.45 * (crossings[j + 1] - ix_r));
        }
        const double r = std::min(r_l, r_r);
        if (r >= 1.2) {
          const double arc_l = ix_l - r;
          const double arc_r = ix_r + r;
          if (arc_l > cur_x + 1e-6) {
            dst->append(QLineF(cur_x, h.y, arc_l, h.y));
          }
          hop_arcs[h.bucket].push_back(
              QRectF(arc_l, h.y - r, arc_r - arc_l, 2.0 * r));
          cur_x = arc_r;
        }
        i = j + 1;
      }
      if (h.x1 > cur_x + 1e-6) {
        dst->append(QLineF(cur_x, h.y, h.x1, h.y));
      }
    }
  }

  if (!valid_segs.isEmpty()) {
    painter.setPen(valid_pen);
    painter.drawLines(valid_segs);
  }
  if (!pending_segs.isEmpty()) {
    painter.setPen(pending_pen);
    painter.drawLines(pending_segs);
  }
  if (!display_segs.isEmpty()) {
    painter.setPen(display_pen);
    painter.drawLines(display_segs);
  }
  if (!warn_segs.isEmpty()) {
    painter.setPen(warn_pen);
    painter.drawLines(warn_segs);
  }
  if (draw_hops &&
      (!hop_arcs[0].empty() || !hop_arcs[1].empty() || !hop_arcs[4].empty())) {
    painter.setRenderHint(QPainter::Antialiasing, true);
    if (!hop_arcs[0].empty()) {
      painter.setPen(valid_pen);
      for (const QRectF& arc_rect : hop_arcs[0]) {
        painter.drawArc(arc_rect, 0, 180 * 16);
      }
    }
    if (!hop_arcs[1].empty()) {
      painter.setPen(display_pen);
      for (const QRectF& arc_rect : hop_arcs[1]) {
        painter.drawArc(arc_rect, 0, 180 * 16);
      }
    }
    if (!hop_arcs[4].empty()) {
      painter.setPen(warn_pen);
      for (const QRectF& arc_rect : hop_arcs[4]) {
        painter.drawArc(arc_rect, 0, 180 * 16);
      }
    }
    painter.setRenderHint(QPainter::Antialiasing, false);
  }

  const auto vis_ids = view.QueryBlocks(vp);
  visible_block_count_ = vis_ids.size();

  if (high_detail) {
    painter.setRenderHint(QPainter::Antialiasing, true);
  }

  QPen block_pen(QColor(72, 86, 106), 1.2);
  block_pen.setCosmetic(true);
  QPen obs_pen(QColor(180, 95, 40), 1.4);
  obs_pen.setCosmetic(true);
  QPen sel_block_pen(QColor(0, 120, 215), 2.2);
  sel_block_pen.setCosmetic(true);
  const QBrush block_brush(QColor(255, 255, 255));
  const QBrush obs_brush(QColor(255, 243, 232));
  const QBrush sel_block_brush(QColor(227, 241, 255));
  const QBrush low_block_brush(QColor(186, 200, 218));
  const QBrush pin_brush(QColor(46, 64, 87));

  QFont label_font = painter.font();
  label_font.setPixelSize(11);
  painter.setFont(label_font);

  if (low_detail) {
    QVector<QRectF> normal_rects;
    QVector<QRectF> sel_rects;
    normal_rects.reserve(static_cast<int>(vis_ids.size()));
    for (localroute::Id id : vis_ids) {
      const auto& b = view.block(id);
      const QRectF r(b.rect.l, b.rect.t, b.rect.r - b.rect.l,
                     b.rect.b - b.rect.t);
      if (selected_blocks_.count(id)) {
        sel_rects.append(r);
      } else {
        normal_rects.append(r);
      }
    }
    painter.setPen(Qt::NoPen);
    if (!normal_rects.isEmpty()) {
      painter.setBrush(low_block_brush);
      painter.drawRects(normal_rects);
    }
    if (!sel_rects.isEmpty()) {
      painter.setBrush(QColor(0, 120, 215));
      painter.drawRects(sel_rects);
    }
  } else {
    for (localroute::Id id : vis_ids) {
      const auto& b = view.block(id);
      const bool selected = selected_blocks_.count(id) != 0;
      const bool is_obstacle = b.pins.empty();
      const QRectF r(b.rect.l, b.rect.t, b.rect.r - b.rect.l,
                     b.rect.b - b.rect.t);
      if (mid_detail) {
        painter.setPen(selected ? sel_block_pen
                                : (is_obstacle ? obs_pen : block_pen));
        painter.setBrush(selected ? sel_block_brush
                                  : (is_obstacle ? obs_brush : block_brush));
        painter.drawRect(r);
      } else {
        painter.setPen(selected ? sel_block_pen
                                : (is_obstacle ? obs_pen : block_pen));
        painter.setBrush(selected ? sel_block_brush
                                  : (is_obstacle ? obs_brush : block_brush));
        painter.drawRoundedRect(r, 4.0, 4.0);
        painter.setPen(is_obstacle ? QColor(145, 70, 24) : QColor(38, 50, 66));
        painter.drawText(
            r, Qt::AlignCenter,
            is_obstacle ? QString("Obs%1").arg(id) : QString("B%1").arg(id));
        for (const auto& p : b.pins) {
          const auto pt = PinPos(b, p.id);
          const bool is_hot =
              (hover_pin_.valid && hover_pin_.block_id == id &&
               hover_pin_.pin_id == p.id) ||
              (drag_kind_ == DragKind::kWiring && wire_src_.block_id == id &&
               wire_src_.pin_id == p.id);
          if (is_hot) {
            painter.setPen(QPen(QColor(255, 255, 255), 1.5));
            painter.setBrush(QColor(0, 168, 107));
            painter.drawEllipse(QPointF(pt.x, pt.y), 5.2, 5.2);
          } else {
            painter.setPen(QPen(QColor(255, 255, 255), 1.0));
            painter.setBrush(pin_brush);
            painter.drawEllipse(QPointF(pt.x, pt.y), 3.2, 3.2);
          }
        }
      }
    }
  }

  painter.setRenderHint(QPainter::Antialiasing, high_detail);
  if (!hi_valid_segs.isEmpty()) {
    painter.setPen(hi_valid_pen);
    painter.drawLines(hi_valid_segs);
  }
  if (!hi_pending_segs.isEmpty()) {
    painter.setPen(hi_pending_pen);
    painter.drawLines(hi_pending_segs);
  }
  if (!hi_display_segs.isEmpty()) {
    painter.setPen(hi_display_pen);
    painter.drawLines(hi_display_segs);
  }
  if (!hi_warn_segs.isEmpty()) {
    painter.setPen(hi_warn_pen);
    painter.drawLines(hi_warn_segs);
  }
  if (draw_hops &&
      (!hop_arcs[2].empty() || !hop_arcs[3].empty() || !hop_arcs[5].empty())) {
    painter.setRenderHint(QPainter::Antialiasing, true);
    if (!hop_arcs[2].empty()) {
      painter.setPen(hi_valid_pen);
      for (const QRectF& arc_rect : hop_arcs[2]) {
        painter.drawArc(arc_rect, 0, 180 * 16);
      }
    }
    if (!hop_arcs[3].empty()) {
      painter.setPen(hi_display_pen);
      for (const QRectF& arc_rect : hop_arcs[3]) {
        painter.drawArc(arc_rect, 0, 180 * 16);
      }
    }
    if (!hop_arcs[5].empty()) {
      painter.setPen(hi_warn_pen);
      for (const QRectF& arc_rect : hop_arcs[5]) {
        painter.drawArc(arc_rect, 0, 180 * 16);
      }
    }
  }
  if (draw_arrows && !arrows.empty()) {
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(Qt::NoPen);
    const double a_len = 7.5 / std::clamp(zoom_, 0.5, 1.5);
    const double a_half_w = 3.2 / std::clamp(zoom_, 0.5, 1.5);
    for (const ArrowHead& ah : arrows) {
      switch (ah.bucket) {
        case 0:
          painter.setBrush(QColor(30, 78, 140));
          break;
        case 1:
          painter.setBrush(highlight_fallback_ ? QColor(230, 126, 34)
                                               : QColor(30, 78, 140));
          break;
        case 2:
          painter.setBrush(QColor(0, 120, 215));
          break;
        case 3:
          painter.setBrush(highlight_fallback_ ? QColor(230, 126, 34)
                                               : QColor(0, 120, 215));
          break;
        default:
          painter.setBrush(QColor(220, 38, 38));
          break;
      }
      const double bx = ah.x - ah.dx * a_len;
      const double by = ah.y - ah.dy * a_len;
      const double px = -ah.dy * a_half_w;
      const double py = ah.dx * a_half_w;
      QPolygonF tri;
      tri << QPointF(ah.x, ah.y) << QPointF(bx + px, by + py)
          << QPointF(bx - px, by - py);
      painter.drawPolygon(tri);
    }
  }
  if (draw_solder && !visible_conn_ids.empty()) {
    const std::vector<localroute::Point> solder_joints =
        view.ComputeSolderJoints(visible_conn_ids);
    if (!solder_joints.empty()) {
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setPen(Qt::NoPen);
      painter.setBrush(QColor(30, 78, 140));
      const double dot_r = 3.2 / std::clamp(zoom_, 0.4, 1.5);
      for (const localroute::Point pt : solder_joints) {
        painter.drawEllipse(QPointF(pt.x, pt.y), dot_r, dot_r);
      }
    }
  }
  visible_conn_count_ = drawn_conns;

  // Smart Guides (alignment lines shown while single-block snapping is active)
  if (!active_guides_.empty()) {
    QPen guide_pen(QColor(64, 158, 255, 220), 1.5, Qt::DashLine);
    guide_pen.setCosmetic(true);
    painter.setPen(guide_pen);
    for (const SmartGuideLine& g : active_guides_) {
      if (g.is_vertical) {
        painter.drawLine(QPointF(g.coord, g.span0), QPointF(g.coord, g.span1));
      } else {
        painter.drawLine(QPointF(g.span0, g.coord), QPointF(g.span1, g.coord));
      }
    }
  }

  // Interactive wiring preview
  if (drag_kind_ == DragKind::kWiring) {
    QPen wire_pen(hover_pin_.valid ? QColor(0, 168, 107) : QColor(0, 120, 215),
                  2.0, Qt::DashLine);
    wire_pen.setCosmetic(true);
    painter.setPen(wire_pen);
    const QPointF mid(drag_current_world_.x(), drag_start_world_.y());
    painter.drawLine(drag_start_world_, mid);
    painter.drawLine(mid, drag_current_world_);
  }

  // Drag-and-drop ghost preview
  if (has_drop_ghost_) {
    const double gw = std::max(
        drop_ghost_tmpl_.width,
        std::max(drop_ghost_tmpl_.top_pins, drop_ghost_tmpl_.bottom_pins) *
                24.0 +
            20.0);
    const double gh = std::max(
        drop_ghost_tmpl_.height,
        std::max(drop_ghost_tmpl_.left_pins, drop_ghost_tmpl_.right_pins) *
                20.0 +
            16.0);
    const QRectF ghost(drop_ghost_world_.x() - gw * 0.5,
                       drop_ghost_world_.y() - gh * 0.5, gw, gh);
    QPen g_pen(QColor(0, 120, 215), 1.6, Qt::DashLine);
    g_pen.setCosmetic(true);
    painter.setPen(g_pen);
    painter.setBrush(QColor(0, 120, 215, 40));
    painter.drawRoundedRect(ghost, 4.0, 4.0);
  }

  if (drag_kind_ == DragKind::kRubberBand) {
    const QRectF rb(std::min(drag_start_world_.x(), drag_current_world_.x()),
                    std::min(drag_start_world_.y(), drag_current_world_.y()),
                    std::abs(drag_current_world_.x() - drag_start_world_.x()),
                    std::abs(drag_current_world_.y() - drag_start_world_.y()));
    QPen rb_pen(QColor(0, 120, 215), 1.4, Qt::DashLine);
    rb_pen.setCosmetic(true);
    painter.setPen(rb_pen);
    painter.setBrush(QColor(0, 120, 215, 36));
    painter.drawRect(rb);
  }

  painter.restore();

  paint_time_ms_ = static_cast<double>(paint_timer.nsecsElapsed()) / 1e6;
  DrawHud(&painter);
}

void CanvasWidget::DrawHud(QPainter* painter) {
  painter->save();
  painter->setRenderHint(QPainter::Antialiasing, true);

  const QRectF hud_rect(12, 12, 420, 152);
  painter->setPen(QPen(QColor(45, 58, 76, 180), 1.0));
  painter->setBrush(QColor(18, 24, 34, 210));
  painter->drawRoundedRect(hud_rect, 8.0, 8.0);

  QFont f("Consolas", 9);
  if (!f.exactMatch()) {
    f = QFont("Monospace", 9);
  }
  painter->setFont(f);
  painter->setPen(QColor(235, 242, 250));

  const char* mode_str = (mode_ == RouteMode::kAsync) ? "Async" : "Sync";
  const char* reroute_str = third_party_reroute_ ? "ON" : "OFF";
  const char* nudge_str = channel_nudging_ ? "ON" : "OFF";
  const QString lines[6] = {
      QString("Scale       : %1 blocks / %2 conns")
          .arg(current_block_count_)
          .arg(current_conn_count_),
      QString("Mode        : %1 | 3rdParty: %2 | Nudge: %3")
          .arg(mode_str)
          .arg(reroute_str)
          .arg(nudge_str),
      QString("FPS / Zoom  : %1 fps | %2x")
          .arg(current_fps_, 0, 'f', 1)
          .arg(zoom_, 0, 'f', 2),
      QString("Route Time  : %1 ms (P99: %2 ms)")
          .arg(route_time_ms_, 0, 'f', 3)
          .arg(RouteP99Ms(), 0, 'f', 3),
      QString("Paint Time  : %1 ms (Vis: %2 blks / %3 conns)")
          .arg(paint_time_ms_, 0, 'f', 3)
          .arg(visible_block_count_)
          .arg(visible_conn_count_),
      QString("Route Status: Valid %1 | Pending %2 | Display %3")
          .arg(status_counts_[static_cast<std::size_t>(
              localroute::Status::kValid)])
          .arg(status_counts_[static_cast<std::size_t>(
              localroute::Status::kPending)])
          .arg(status_counts_[static_cast<std::size_t>(
                   localroute::Status::kDisplayOnly)] +
               status_counts_[static_cast<std::size_t>(
                   localroute::Status::kFailed)]),
  };

  int y = 34;
  for (const QString& line : lines) {
    painter->drawText(24, y, line);
    y += 22;
  }
  painter->restore();
}
