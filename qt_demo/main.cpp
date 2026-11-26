#include "MainWindow.h"
#include "topology_generator.h"
#include <QApplication>
#include <QElapsedTimer>
#include <QPixmap>
#include <QTimer>
#include <iostream>
#include <vector>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    bool smoke = false;
    for (int i = 1; i < argc; ++i) {
        if (QString::fromLocal8Bit(argv[i]) == "--smoke-bench")
            smoke = true;
    }

    MainWindow win(10000);
    if (!smoke) {
        win.show();
        return app.exec();
    }

    // Offscreen / short-lived smoke test measuring real 10k Paint & Route times in Sync & Async.
    win.show();
    QApplication::processEvents();
    auto *canvas = win.canvasWidget();

    // Measure Async cold start + drag + paint
    QPixmap scratch(canvas->size());
    canvas->render(&scratch);
    double asyncColdRouteMs = canvas->lastRouteMs();
    double asyncColdPaintMs = canvas->lastPaintMs();
    size_t asyncVisBlocks = canvas->lastVisibleBlocks();
    size_t asyncVisConns = canvas->lastVisibleConns();

    canvas->stepSmokeDrag({5000}, 12.0, 8.0);
    canvas->render(&scratch);
    double asyncSingleRouteMs = canvas->lastRouteMs();
    double asyncSinglePaintMs = canvas->lastPaintMs();

    Topology top = generate(10000);
    std::vector<localroute::Id> box20;
    for (localroute::Id i = 0; i < 20; ++i)
        box20.push_back((i / 10) * top.side + i % 10);
    canvas->stepSmokeDrag(box20, 12.0, 8.0);
    canvas->render(&scratch);
    double asyncBox20RouteMs = canvas->lastRouteMs();
    double asyncBox20PaintMs = canvas->lastPaintMs();

    // Switch to Sync and measure 10k cold start + single drag + 20-block drag + Paint Time
    canvas->setRouteMode(CanvasWidget::RouteMode::Sync);
    canvas->render(&scratch);
    double syncColdRouteMs = canvas->lastRouteMs();
    double syncColdPaintMs = canvas->lastPaintMs();

    canvas->stepSmokeDrag({5000}, 12.0, 8.0);
    canvas->render(&scratch);
    double syncSingleRouteMs = canvas->lastRouteMs();
    double syncSinglePaintMs = canvas->lastPaintMs();

    canvas->stepSmokeDrag(box20, 12.0, 8.0);
    canvas->render(&scratch);
    double syncBox20RouteMs = canvas->lastRouteMs();
    double syncBox20PaintMs = canvas->lastPaintMs();

    // Also verify low-zoom Fit All paint time
    canvas->fitAll();
    canvas->render(&scratch);
    double fitAllPaintMs = canvas->lastPaintMs();
    size_t fitVisBlocks = canvas->lastVisibleBlocks();
    size_t fitVisConns = canvas->lastVisibleConns();

    std::cout << "SMOKE_PASS"
              << " async_cold_route_ms=" << asyncColdRouteMs
              << " async_cold_paint_ms=" << asyncColdPaintMs
              << " async_vis=" << asyncVisBlocks << "/" << asyncVisConns
              << " async_single_route_ms=" << asyncSingleRouteMs
              << " async_single_paint_ms=" << asyncSinglePaintMs
              << " async_box20_route_ms=" << asyncBox20RouteMs
              << " async_box20_paint_ms=" << asyncBox20PaintMs
              << " sync_cold_route_ms=" << syncColdRouteMs
              << " sync_cold_paint_ms=" << syncColdPaintMs
              << " sync_single_route_ms=" << syncSingleRouteMs
              << " sync_single_paint_ms=" << syncSinglePaintMs
              << " sync_box20_route_ms=" << syncBox20RouteMs
              << " sync_box20_paint_ms=" << syncBox20PaintMs
              << " fit_all_paint_ms=" << fitAllPaintMs
              << " fit_vis=" << fitVisBlocks << "/" << fitVisConns
              << std::endl;

    return 0;
}
