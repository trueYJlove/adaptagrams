#include "canvas_widget.h"

#include <QKeyEvent>
#include <QLineF>
#include <QMouseEvent>
#include <QPainter>
#include <QVector>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
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

}  // namespace

CanvasWidget::CanvasWidget(QWidget* parent) : QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
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

localroute::LocalOrthogonalRouter CanvasWidget::ExtractModelCopy() const {
  const auto& view = ActiveView();
  localroute::Config cfg = view.config();
  cfg.reroute_third_party_blocked = third_party_reroute_;
  localroute::LocalOrthogonalRouter copy(cfg);
  for (const auto& kv : view.all_blocks()) {
    copy.AddBlock(kv.second);
  }
  for (const auto& kv : view.all_conns()) {
    copy.AddConnector(kv.second);
  }
  return copy;
}

void CanvasWidget::LoadTopology(unsigned block_count) {
  async_timer_.stop();
  async_router_.reset();
  sync_router_.reset();
  selected_blocks_.clear();
  highlighted_conns_.clear();
  route_samples_ms_.clear();
  drag_kind_ = DragKind::kNone;

  const localroute::Topology top = localroute::GenerateTopology(block_count);
  current_block_count_ = static_cast<unsigned>(top.blocks.size());
  current_conn_count_ = static_cast<unsigned>(top.conns.size());

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto& b : top.blocks) {
    min_x = std::min(min_x, b.rect.l);
    min_y = std::min(min_y, b.rect.t);
    max_x = std::max(max_x, b.rect.r);
    max_y = std::max(max_y, b.rect.b);
  }
  world_bounds_ = {min_x, min_y, max_x, max_y};

  localroute::Config cfg;
  cfg.reroute_third_party_blocked = third_party_reroute_;
  localroute::LocalOrthogonalRouter initial(cfg);
  for (const auto& b : top.blocks) {
    initial.AddBlock(b);
  }
  for (const auto& c : top.conns) {
    initial.AddConnector(c);
  }

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

  RebuildRouteCache();
  ResetLocalView();

  if (mode_ == RouteMode::kAsync && async_router_) {
    QElapsedTimer start_timer;
    start_timer.start();
    async_router_->Prioritize(WorldViewportRect(), {});
    async_router_->StartAll();
    setup_ms += static_cast<double>(start_timer.nsecsElapsed()) / 1e6;
    RebuildRouteCache();
    async_timer_.start();
  }

  route_time_ms_ = setup_ms;
  RecordRouteSample(setup_ms);
  update();
  emit StatusChanged();
}

void CanvasWidget::SetRouteMode(RouteMode new_mode) {
  if (mode_ == new_mode) {
    return;
  }
  async_timer_.stop();
  localroute::LocalOrthogonalRouter model = ExtractModelCopy();
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
    async_timer_.start();
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
    async_timer_.start();
  }
  RecordRouteSample(route_time_ms_);
  RebuildRouteCache();
  update();
  emit StatusChanged();
}

void CanvasWidget::FitAll() {
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
    pan_offset_ = QPointF(40.0, 40.0);
  }
  UpdateViewportPriority();
  update();
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

void CanvasWidget::StepSmokeDrag(const std::vector<localroute::Id>& ids,
                                 double dx, double dy) {
  if (ids.empty()) {
    return;
  }
  const auto& view = ActiveView();
  std::vector<std::pair<localroute::Id, localroute::Rect>> moves;
  moves.reserve(ids.size());
  for (localroute::Id id : ids) {
    const localroute::Rect r = view.block(id).rect;
    moves.push_back({id, {r.l + dx, r.t + dy, r.r + dx, r.b + dy}});
  }
  QElapsedTimer timer;
  timer.start();
  if (mode_ == RouteMode::kSync && sync_router_) {
    sync_router_->MoveBlocksBatch(moves);
    const auto changed = sync_router_->RerouteAffected(ids);
    route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
    UpdateRouteCacheForIds(changed);
  } else if (mode_ == RouteMode::kAsync && async_router_) {
    async_router_->Move(moves);
    std::vector<localroute::Id> updated;
    async_router_->Poll(128, &updated);
    route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
    for (localroute::Id id : ids) {
      UpdateRouteBoundsFromBlock(id);
    }
    if (!updated.empty()) {
      UpdateRouteCacheForIds(updated);
    }
  }
  RecordRouteSample(route_time_ms_);
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
  const std::size_t n = async_router_->Poll(batch_size, &updated);
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

  const QPointF world_pos = ScreenToWorld(event->pos());
  const localroute::Id hit = HitTestBlock(world_pos);
  const auto& view = ActiveView();

  if (hit != std::numeric_limits<localroute::Id>::max()) {
    if (event->modifiers() & Qt::ControlModifier) {
      if (selected_blocks_.count(hit)) {
        selected_blocks_.erase(hit);
      } else {
        selected_blocks_.insert(hit);
      }
    } else if (!selected_blocks_.count(hit)) {
      selected_blocks_.clear();
      selected_blocks_.insert(hit);
    }
    drag_kind_ = DragKind::kMoveBlocks;
    drag_start_world_ = world_pos;
    drag_initial_rects_.clear();
    highlighted_conns_.clear();
    for (localroute::Id id : selected_blocks_) {
      drag_initial_rects_.emplace(id, view.block(id).rect);
      for (localroute::Id cid : view.adjacent_conns(id)) {
        highlighted_conns_.insert(cid);
      }
    }
    UpdateViewportPriority();
    update();
  } else {
    if (!(event->modifiers() & Qt::ControlModifier)) {
      selected_blocks_.clear();
    }
    highlighted_conns_.clear();
    drag_kind_ = DragKind::kRubberBand;
    drag_start_world_ = world_pos;
    drag_current_world_ = world_pos;
    update();
  }
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

  if (drag_kind_ == DragKind::kRubberBand) {
    drag_current_world_ = ScreenToWorld(pos);
    update();
    return;
  }

  if (drag_kind_ == DragKind::kMoveBlocks && !drag_initial_rects_.empty()) {
    const QPointF world_pos = ScreenToWorld(pos);
    const double dx = world_pos.x() - drag_start_world_.x();
    const double dy = world_pos.y() - drag_start_world_.y();
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
      async_router_->Poll(128, &updated);
      route_time_ms_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;
      for (localroute::Id id : moved_ids) {
        UpdateRouteBoundsFromBlock(id);
      }
      if (!updated.empty()) {
        UpdateRouteCacheForIds(updated);
      }
      if (!async_timer_.isActive()) {
        async_timer_.start();
      }
    }
    RecordRouteSample(route_time_ms_);
    update();
    emit StatusChanged();
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
    return;
  }

  if (drag_kind_ == DragKind::kMoveBlocks) {
    drag_kind_ = DragKind::kNone;
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
    drag_initial_rects_.clear();
    update();
    emit StatusChanged();
  }
}

void CanvasWidget::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Space && !event->isAutoRepeat()) {
    space_pressed_ = true;
    if (drag_kind_ == DragKind::kNone) {
      setCursor(Qt::OpenHandCursor);
    }
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

  const bool high_detail = zoom_ >= 0.4;
  const bool mid_detail = zoom_ >= 0.15 && zoom_ < 0.4;
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
  QPen display_pen(QColor(230, 126, 34), 1.5);
  display_pen.setCosmetic(true);
  QPen hi_valid_pen(QColor(0, 120, 215), 2.2);
  hi_valid_pen.setCosmetic(true);
  QPen hi_pending_pen(QColor(0, 120, 215), 1.8, Qt::DashLine);
  hi_pending_pen.setCosmetic(true);

  QVector<QLineF> valid_segs;
  QVector<QLineF> pending_segs;
  QVector<QLineF> display_segs;
  valid_segs.reserve(2048);
  pending_segs.reserve(1024);
  display_segs.reserve(128);
  std::vector<localroute::Id> top_highlight;
  top_highlight.reserve(highlighted_conns_.size());

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

  const auto& all_blocks = view.all_blocks();
  const auto& all_conns = view.all_conns();
  std::size_t drawn_conns = 0;
  for (const auto& entry : route_cache_) {
    if (highlighted_conns_.count(entry.id)) {
      top_highlight.push_back(entry.id);
      continue;
    }
    if (!entry.bounds.Intersects(vp)) {
      continue;
    }
    ++drawn_conns;
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
      auto& dst = (entry.status == localroute::Status::kValid) ? valid_segs
                                                               : display_segs;
      const auto& pts = rit->second.points;
      for (std::size_t j = 1; j < pts.size(); ++j) {
        append_clipped(&dst, pts[j - 1], pts[j]);
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

  const auto vis_ids = view.QueryBlocks(vp);
  visible_block_count_ = vis_ids.size();

  if (high_detail) {
    painter.setRenderHint(QPainter::Antialiasing, true);
  }

  QPen block_pen(QColor(72, 86, 106), 1.2);
  block_pen.setCosmetic(true);
  QPen sel_block_pen(QColor(0, 120, 215), 2.2);
  sel_block_pen.setCosmetic(true);
  const QBrush block_brush(QColor(255, 255, 255));
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
      const QRectF r(b.rect.l, b.rect.t, b.rect.r - b.rect.l,
                     b.rect.b - b.rect.t);
      if (mid_detail) {
        painter.setPen(selected ? sel_block_pen : block_pen);
        painter.setBrush(selected ? sel_block_brush : block_brush);
        painter.drawRect(r);
      } else {
        painter.setPen(selected ? sel_block_pen : block_pen);
        painter.setBrush(selected ? sel_block_brush : block_brush);
        painter.drawRoundedRect(r, 4.0, 4.0);
        painter.setPen(QColor(38, 50, 66));
        painter.drawText(r, Qt::AlignCenter, QString("B%1").arg(id));
        painter.setPen(Qt::NoPen);
        painter.setBrush(pin_brush);
        for (const auto& p : b.pins) {
          const auto pt = PinPos(b, p.id);
          painter.drawEllipse(QPointF(pt.x, pt.y), 2.8, 2.8);
        }
      }
    }
  }

  painter.setRenderHint(QPainter::Antialiasing, high_detail);
  QVector<QLineF> hi_valid_segs;
  QVector<QLineF> hi_pending_segs;
  QVector<QLineF> hi_display_segs;
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
      auto& dst = (rit->second.status == localroute::Status::kValid)
                      ? hi_valid_segs
                      : hi_display_segs;
      const auto& pts = rit->second.points;
      for (std::size_t j = 1; j < pts.size(); ++j) {
        append_clipped(&dst, pts[j - 1], pts[j]);
      }
    }
  }
  if (!hi_valid_segs.isEmpty()) {
    painter.setPen(hi_valid_pen);
    painter.drawLines(hi_valid_segs);
  }
  if (!hi_pending_segs.isEmpty()) {
    painter.setPen(hi_pending_pen);
    painter.drawLines(hi_pending_segs);
  }
  if (!hi_display_segs.isEmpty()) {
    painter.setPen(display_pen);
    painter.drawLines(hi_display_segs);
  }
  visible_conn_count_ = drawn_conns;

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

  const QRectF hud_rect(12, 12, 370, 152);
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
  const QString lines[6] = {
      QString("Scale       : %1 blocks / %2 conns")
          .arg(current_block_count_)
          .arg(current_conn_count_),
      QString("Mode        : %1 | ThirdPartyReroute: %2")
          .arg(mode_str)
          .arg(reroute_str),
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
