#include <QApplication>
#include <QPixmap>
#include <cassert>
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
            << " custom_wire1_pts=" << r1.points.size()
            << " custom_wire2_pts=" << r2.points.size() << std::endl;

  return 0;
}
