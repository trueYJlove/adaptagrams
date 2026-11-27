#include <QApplication>
#include <QDir>
#include <QFile>
#include <QPixmap>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include "main_window.h"
#include "topology_generator.h"

int main(int argc, char* argv[]) {
  bool smoke = false;
  QString smoke_out_path;
  for (int i = 1; i < argc; ++i) {
    const QString arg = QString::fromLocal8Bit(argv[i]);
    if (arg == "--smoke-bench") {
      smoke = true;
    } else if (arg == "--smoke-out" && i + 1 < argc) {
      smoke_out_path = QString::fromLocal8Bit(argv[++i]);
    }
  }

#ifdef _WIN32
  if (smoke) {
    HANDLE h_out = GetStdHandle(STD_OUTPUT_HANDLE);
    const bool out_redirected =
        (h_out != nullptr && h_out != INVALID_HANDLE_VALUE &&
         GetFileType(h_out) != FILE_TYPE_UNKNOWN);
    if (!out_redirected && AttachConsole(ATTACH_PARENT_PROCESS)) {
      FILE* fp = nullptr;
      freopen_s(&fp, "CONOUT$", "w", stdout);
      freopen_s(&fp, "CONOUT$", "w", stderr);
    }
  }
#endif

  QApplication app(argc, argv);

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

  // Verify shared-trunk drag + user_modified preservation + collision warning
  // + AutoRoute recovery on Cascade preset (B8 fans out to B11 & B12 on conns
  // 3 and 4).
  const std::vector<localroute::Point> frozen_1 =
      canvas->router_view().route(1).points;
  const std::vector<localroute::Point> frozen_2 =
      canvas->router_view().route(2).points;
  if (!canvas->StepSmokeSegmentDrag(3, 1, 210.0)) {
    throw std::runtime_error("smoke shared-trunk segment drag failed");
  }
  canvas->render(&scratch);
  if (!canvas->router_view().route(3).user_modified ||
      !canvas->router_view().route(4).user_modified ||
      std::abs(canvas->router_view().route(3).points[1].x - 210.0) > 1e-6 ||
      std::abs(canvas->router_view().route(4).points[1].x - 210.0) > 1e-6) {
    throw std::runtime_error("smoke shared-trunk did not follow together");
  }
  if (!(canvas->router_view().route(1).points == frozen_1) ||
      !(canvas->router_view().route(2).points == frozen_2)) {
    throw std::runtime_error(
        "smoke segment drag pushed unrelated neighbor wires");
  }
  // Verify SetEnableChannelNudging(false) preserves user_modified routes.
  canvas->SetEnableChannelNudging(false);
  canvas->render(&scratch);
  if (!canvas->router_view().route(3).user_modified ||
      !canvas->router_view().route(4).user_modified ||
      std::abs(canvas->router_view().route(3).points[1].x - 210.0) > 1e-6 ||
      std::abs(canvas->router_view().route(4).points[1].x - 210.0) > 1e-6) {
    throw std::runtime_error(
        "smoke disabling channel nudging dropped user_modified routes");
  }
  canvas->SetEnableChannelNudging(true);
  canvas->render(&scratch);
  // Dragging onto B11 obstacle (x=290) sets user_modified && kDisplayOnly.
  if (!canvas->StepSmokeSegmentDrag(3, 1, 290.0)) {
    throw std::runtime_error("smoke collision segment drag failed");
  }
  if (!canvas->router_view().route(3).user_modified ||
      canvas->router_view().route(3).status !=
          localroute::Status::kDisplayOnly) {
    throw std::runtime_error(
        "smoke obstacle-collision segment drag did not mark DisplayOnly");
  }
  const auto auto_routed = canvas->StepSmokeAutoRoute({3, 4});
  canvas->render(&scratch);
  if (auto_routed.size() < 2 || canvas->router_view().route(3).user_modified ||
      canvas->router_view().route(3).status != localroute::Status::kValid ||
      canvas->router_view().route(4).status != localroute::Status::kValid) {
    throw std::runtime_error("smoke auto-route did not clear user_modified");
  }

  // Verify 10k Async incremental Undo / Redo (< 5.0 ms gate, expect < 1.0 ms)
  // and viewport SVG export.
  canvas->SetRouteMode(CanvasWidget::RouteMode::kAsync);
  canvas->LoadTopology(10000);
  canvas->render(&scratch);
  const localroute::Rect orig_5000 = canvas->router_view().block(5000).rect;
  canvas->StepSmokeDrag({5000}, 15.0, 10.0);
  bool undo_ok = false;
  const double undo_ms = canvas->StepSmokeUndo(&undo_ok);
  if (!undo_ok ||
      std::abs(canvas->router_view().block(5000).rect.l - orig_5000.l) > 1e-6 ||
      undo_ms >= 5.0) {
    throw std::runtime_error("smoke 10k async undo failed or exceeded 5ms");
  }
  bool redo_ok = false;
  const double redo_ms = canvas->StepSmokeRedo(&redo_ok);
  if (!redo_ok ||
      std::abs(canvas->router_view().block(5000).rect.l -
               (orig_5000.l + 15.0)) > 1e-6 ||
      redo_ms >= 5.0) {
    throw std::runtime_error("smoke 10k async redo failed or exceeded 5ms");
  }
  const QString svg_tmp = QDir::tempPath() + "/localroute_smoke_viewport.svg";
  QFile::remove(svg_tmp);
  if (!canvas->ExportViewportSvg(svg_tmp)) {
    throw std::runtime_error("smoke SVG export failed");
  }
  QFile svg_file(svg_tmp);
  if (!svg_file.open(QIODevice::ReadOnly) ||
      !svg_file.read(256).contains("<svg")) {
    throw std::runtime_error("smoke SVG content invalid");
  }
  svg_file.close();
  QFile::remove(svg_tmp);

  const QString svg_utf8_tmp =
      QDir::tempPath() + QString::fromUtf8("/localroute_测试_🔌.svg");
  QFile::remove(svg_utf8_tmp);
  if (!canvas->ExportViewportSvg(svg_utf8_tmp)) {
    throw std::runtime_error("smoke UTF-8 SVG export failed");
  }
  QFile svg_utf8_file(svg_utf8_tmp);
  if (!svg_utf8_file.open(QIODevice::ReadOnly) ||
      !svg_utf8_file.read(256).contains("<svg")) {
    throw std::runtime_error("smoke UTF-8 SVG content invalid");
  }
  svg_utf8_file.close();
  QFile::remove(svg_utf8_tmp);

  const std::string summary =
      std::string("SMOKE_PASS async_cold_route_ms=") +
      std::to_string(async_cold_route_ms) +
      " async_cold_paint_ms=" + std::to_string(async_cold_paint_ms) +
      " async_vis=" + std::to_string(async_vis_blocks) + "/" +
      std::to_string(async_vis_conns) +
      " async_single_route_ms=" + std::to_string(async_single_route_ms) +
      " async_single_paint_ms=" + std::to_string(async_single_paint_ms) +
      " async_box20_route_ms=" + std::to_string(async_box20_route_ms) +
      " async_box20_paint_ms=" + std::to_string(async_box20_paint_ms) +
      " sync_cold_route_ms=" + std::to_string(sync_cold_route_ms) +
      " sync_cold_paint_ms=" + std::to_string(sync_cold_paint_ms) +
      " sync_single_route_ms=" + std::to_string(sync_single_route_ms) +
      " sync_single_paint_ms=" + std::to_string(sync_single_paint_ms) +
      " sync_box20_route_ms=" + std::to_string(sync_box20_route_ms) +
      " sync_box20_paint_ms=" + std::to_string(sync_box20_paint_ms) +
      " fit_all_paint_ms=" + std::to_string(fit_all_paint_ms) +
      " fit_vis=" + std::to_string(fit_vis_blocks) + "/" +
      std::to_string(fit_vis_conns) +
      " custom_wire1_pts=" + std::to_string(cw1) +
      " custom_wire2_pts=" + std::to_string(cw2) +
      " preset_bundle_min_gap=" + std::to_string(bundle_min_gap) +
      " preset_cascade_min_gap=" + std::to_string(cascade_min_gap) +
      " preset_cascade_off_gap=" + std::to_string(off_gap) +
      " undo_ms=" + std::to_string(undo_ms) +
      " redo_ms=" + std::to_string(redo_ms);
  std::cout << summary << std::endl;
  if (!smoke_out_path.isEmpty()) {
    std::ofstream fout(smoke_out_path.toStdString());
    fout << summary << '\n';
  }

  return 0;
}
