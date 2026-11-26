#include <QApplication>
#include <QPixmap>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "main_window.h"
#include "topology_generator.h"

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);

  bool smoke = false;
  for (int i = 1; i < argc; ++i) {
    if (QString::fromLocal8Bit(argv[i]) == "--smoke-bench") {
      smoke = true;
    }
  }

  MainWindow win(10000);
  if (!smoke) {
    win.show();
    return app.exec();
  }

  win.show();
  QApplication::processEvents();
  CanvasWidget* canvas = win.canvas_widget();

  QPixmap scratch(canvas->size());
  canvas->render(&scratch);
  const double async_cold_route_ms = canvas->last_route_ms();
  const double async_cold_paint_ms = canvas->last_paint_ms();
  const std::size_t async_vis_blocks = canvas->last_visible_blocks();
  const std::size_t async_vis_conns = canvas->last_visible_conns();

  canvas->StepSmokeDrag({5000}, 12.0, 8.0);
  canvas->render(&scratch);
  const double async_single_route_ms = canvas->last_route_ms();
  const double async_single_paint_ms = canvas->last_paint_ms();

  const localroute::Topology top = localroute::GenerateTopology(10000);
  std::vector<localroute::Id> box20;
  for (localroute::Id i = 0; i < 20; ++i) {
    box20.push_back((i / 10) * top.side + i % 10);
  }
  canvas->StepSmokeDrag(box20, 12.0, 8.0);
  canvas->render(&scratch);
  const double async_box20_route_ms = canvas->last_route_ms();
  const double async_box20_paint_ms = canvas->last_paint_ms();

  canvas->SetRouteMode(CanvasWidget::RouteMode::kSync);
  canvas->render(&scratch);
  const double sync_cold_route_ms = canvas->last_route_ms();
  const double sync_cold_paint_ms = canvas->last_paint_ms();

  canvas->StepSmokeDrag({5000}, 12.0, 8.0);
  canvas->render(&scratch);
  const double sync_single_route_ms = canvas->last_route_ms();
  const double sync_single_paint_ms = canvas->last_paint_ms();

  canvas->StepSmokeDrag(box20, 12.0, 8.0);
  canvas->render(&scratch);
  const double sync_box20_route_ms = canvas->last_route_ms();
  const double sync_box20_paint_ms = canvas->last_paint_ms();

  canvas->FitAll();
  canvas->render(&scratch);
  const double fit_all_paint_ms = canvas->last_paint_ms();
  const std::size_t fit_vis_blocks = canvas->last_visible_blocks();
  const std::size_t fit_vis_conns = canvas->last_visible_conns();

  // Verify interactive modeling workflow: blank canvas -> drop blocks -> edit
  // ports -> manual wire around obstacle -> verify valid obstacle-avoiding
  // route.
  canvas->LoadTopology(0);
  const BlockTemplate basic{"Basic", 100.0, 60.0, 1, 1, 0, 0};
  const BlockTemplate obstacle{"Obstacle", 100.0, 120.0, 0, 0, 0, 0};
  const localroute::Id b1 =
      canvas->AddBlockFromTemplate(basic, QPointF(-200.0, 0.0));
  const localroute::Id b2 =
      canvas->AddBlockFromTemplate(basic, QPointF(200.0, 0.0));
  canvas->AddBlockFromTemplate(obstacle, QPointF(0.0, 0.0));
  canvas->SetBlockPorts(b1, 1, 2, 0, 0);
  canvas->SetBlockPorts(b2, 2, 1, 0, 0);
  const localroute::Id w1 = canvas->AddWire(b1, 200, b2, 100);
  const localroute::Id w2 = canvas->AddWire(b1, 201, b2, 101);
  canvas->render(&scratch);
  const auto& r1 = canvas->router_view().route(w1);
  const auto& r2 = canvas->router_view().route(w2);
  if (r1.status != localroute::Status::kValid || r1.points.size() < 4 ||
      r2.status != localroute::Status::kValid || r2.points.size() < 4) {
    throw std::runtime_error("interactive modeling smoke check failed");
  }
  const std::size_t cw1 = r1.points.size();
  const std::size_t cw2 = r2.points.size();

  // Verify Preset 1: B1-B2 10-wire staggered parallel bundle nudging.
  canvas->LoadPresetParallelBundle();
  canvas->render(&scratch);
  std::vector<double> bundle_xs;
  for (localroute::Id cid = 1; cid <= 10; ++cid) {
    const auto& rt = canvas->router_view().route(cid);
    if (rt.status != localroute::Status::kValid || rt.points.size() != 4) {
      throw std::runtime_error("preset parallel bundle smoke check failed");
    }
    bundle_xs.push_back(rt.points[1].x);
  }
  std::sort(bundle_xs.begin(), bundle_xs.end());
  double bundle_min_gap = 1e9;
  for (std::size_t i = 1; i < bundle_xs.size(); ++i) {
    bundle_min_gap = std::min(bundle_min_gap, bundle_xs[i] - bundle_xs[i - 1]);
  }
  if (bundle_min_gap < 4.0) {
    throw std::runtime_error("preset parallel bundle min gap < 4px");
  }

  // Verify Preset 2: B9/B3/B8 cascade routing + live toggle off/on.
  canvas->LoadPresetCascade();
  canvas->render(&scratch);
  std::vector<double> cascade_xs;
  for (localroute::Id cid = 1; cid <= 4; ++cid) {
    const auto& rt = canvas->router_view().route(cid);
    if (rt.status != localroute::Status::kValid || rt.points.size() > 4) {
      throw std::runtime_error("preset cascade smoke check failed");
    }
    if (cid <= 3) {
      cascade_xs.push_back(rt.points[1].x);
    }
  }
  if (std::abs(canvas->router_view().route(3).points[1].x -
               canvas->router_view().route(4).points[1].x) > 1e-6) {
    throw std::runtime_error("preset cascade fan-out trunk split unexpectedly");
  }
  std::sort(cascade_xs.begin(), cascade_xs.end());
  const double cascade_min_gap =
      std::min(cascade_xs[1] - cascade_xs[0], cascade_xs[2] - cascade_xs[1]);
  if (cascade_min_gap < 4.0) {
    throw std::runtime_error("preset cascade min gap < 4px");
  }
  canvas->SetEnableChannelNudging(false);
  canvas->render(&scratch);
  const double off_gap = std::abs(canvas->router_view().route(1).points[1].x -
                                  canvas->router_view().route(2).points[1].x);
  if (off_gap > 1e-6) {
    throw std::runtime_error("preset cascade off toggle failed");
  }
  canvas->SetEnableChannelNudging(true);
  canvas->render(&scratch);

  // Verify dragging B3 in qt_demo does not move anchored B9->B10.
  const std::vector<localroute::Point> frozen_b9_qt =
      canvas->router_view().route(1).points;
  canvas->StepSmokeDrag({3}, 80.0, 0.0);
  canvas->render(&scratch);
  if (!(canvas->router_view().route(1).points == frozen_b9_qt) ||
      canvas->router_view().route(2).status != localroute::Status::kValid) {
    throw std::runtime_error("preset cascade drag anchor invariance failed");
  }

  // Verify solder joints on Cascade preset (B8 fans out to B11 and B12).
  const std::vector<localroute::Point> cascade_joints =
      canvas->router_view().ComputeSolderJoints({1, 2, 3, 4});
  if (cascade_joints.empty()) {
    throw std::runtime_error("preset cascade solder joint missing");
  }

  // Verify segment drag + user_modified preservation + AutoRoute recovery.
  if (!canvas->StepSmokeSegmentDrag(1, 1, 210.0)) {
    throw std::runtime_error("smoke segment drag failed");
  }
  canvas->render(&scratch);
  if (!canvas->router_view().route(1).user_modified ||
      std::abs(canvas->router_view().route(1).points[1].x - 210.0) > 1e-6) {
    throw std::runtime_error("smoke segment drag did not lock route");
  }
  canvas->StepSmokeDrag({9}, 0.0, 10.0);
  canvas->render(&scratch);
  if (!canvas->router_view().route(1).user_modified ||
      std::abs(canvas->router_view().route(1).points[1].x - 210.0) > 1e-6) {
    throw std::runtime_error("smoke segment drag lock lost on block move");
  }
  const auto auto_routed = canvas->StepSmokeAutoRoute({1});
  canvas->render(&scratch);
  if (auto_routed.empty() || canvas->router_view().route(1).user_modified ||
      canvas->router_view().route(1).status != localroute::Status::kValid) {
    throw std::runtime_error("smoke auto-route did not clear user_modified");
  }

  std::cout << "SMOKE_PASS" << " async_cold_route_ms=" << async_cold_route_ms
            << " async_cold_paint_ms=" << async_cold_paint_ms
            << " async_vis=" << async_vis_blocks << "/" << async_vis_conns
            << " async_single_route_ms=" << async_single_route_ms
            << " async_single_paint_ms=" << async_single_paint_ms
            << " async_box20_route_ms=" << async_box20_route_ms
            << " async_box20_paint_ms=" << async_box20_paint_ms
            << " sync_cold_route_ms=" << sync_cold_route_ms
            << " sync_cold_paint_ms=" << sync_cold_paint_ms
            << " sync_single_route_ms=" << sync_single_route_ms
            << " sync_single_paint_ms=" << sync_single_paint_ms
            << " sync_box20_route_ms=" << sync_box20_route_ms
            << " sync_box20_paint_ms=" << sync_box20_paint_ms
            << " fit_all_paint_ms=" << fit_all_paint_ms
            << " fit_vis=" << fit_vis_blocks << "/" << fit_vis_conns
            << " custom_wire1_pts=" << cw1 << " custom_wire2_pts=" << cw2
            << " preset_bundle_min_gap=" << bundle_min_gap
            << " preset_cascade_min_gap=" << cascade_min_gap
            << " preset_cascade_off_gap=" << off_gap << std::endl;

  return 0;
}
