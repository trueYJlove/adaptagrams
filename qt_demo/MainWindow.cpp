#include "MainWindow.h"
#include <QAction>
#include <QLabel>
#include <QPushButton>
#include <QStatusBar>
#include <QToolBar>

MainWindow::MainWindow(unsigned initialScale, QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("Local Orthogonal Router - 万级正交布线实时交互演示");
    resize(1360, 860);

    canvas = new CanvasWidget(this);
    setCentralWidget(canvas);

    QToolBar *toolbar = addToolBar("Controls");
    toolbar->setMovable(false);

    toolbar->addWidget(new QLabel(" 拓扑规模: ", toolbar));
    scaleCombo = new QComboBox(toolbar);
    scaleCombo->addItem("1,000 模块 (2k 线)", 1000);
    scaleCombo->addItem("3,000 模块 (6k 线)", 3000);
    scaleCombo->addItem("5,000 模块 (10k 线)", 5000);
    scaleCombo->addItem("10,000 模块 (20k 线)", 10000);
    int initIdx = scaleCombo->findData(initialScale);
    scaleCombo->setCurrentIndex(initIdx >= 0 ? initIdx : 3);
    toolbar->addWidget(scaleCombo);

    toolbar->addSeparator();
    toolbar->addWidget(new QLabel(" 路由模式: ", toolbar));
    modeCombo = new QComboBox(toolbar);
    modeCombo->addItem("Async (异步)", int(CanvasWidget::RouteMode::Async));
    modeCombo->addItem("Sync (同步)", int(CanvasWidget::RouteMode::Sync));
    toolbar->addWidget(modeCombo);

    toolbar->addSeparator();
    thirdPartyCheck = new QCheckBox("第三方压线重算 (默认关闭)", toolbar);
    thirdPartyCheck->setChecked(false);
    toolbar->addWidget(thirdPartyCheck);

    toolbar->addSeparator();
    auto *fitBtn = new QPushButton("适应全图 (F)", toolbar);
    auto *resetBtn = new QPushButton("局部 100% 视角 (R)", toolbar);
    auto *routeAllBtn = new QPushButton("重新全图布线", toolbar);
    toolbar->addWidget(fitBtn);
    toolbar->addWidget(resetBtn);
    toolbar->addWidget(routeAllBtn);

    connect(scaleCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onScaleChanged);
    connect(modeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onModeChanged);
    connect(thirdPartyCheck, &QCheckBox::toggled, this, &MainWindow::onThirdPartyToggled);
    connect(fitBtn, &QPushButton::clicked, canvas, &CanvasWidget::fitAll);
    connect(resetBtn, &QPushButton::clicked, canvas, &CanvasWidget::resetLocalView);
    connect(routeAllBtn, &QPushButton::clicked, canvas, &CanvasWidget::routeAll);
    connect(canvas, &CanvasWidget::statusChanged, this, &MainWindow::updateStatusSummary);

    canvas->loadTopology(scaleCombo->currentData().toUInt());
    updateStatusSummary();
}

void MainWindow::onScaleChanged(int index) {
    unsigned n = scaleCombo->itemData(index).toUInt();
    canvas->loadTopology(n);
}

void MainWindow::onModeChanged(int index) {
    auto m = static_cast<CanvasWidget::RouteMode>(modeCombo->itemData(index).toInt());
    canvas->setRouteMode(m);
}

void MainWindow::onThirdPartyToggled(bool checked) {
    canvas->setRerouteThirdPartyBlocked(checked);
}

void MainWindow::updateStatusSummary() {
    statusBar()->showMessage(
        QString("滚轮缩放 | 中键或空格+左键平移 | 左键拖拽模块或框选 | F 全图 / R 局部100% | 当前: %1 模块 / %2 连线 | Route %3 ms | Paint %4 ms")
            .arg(canvas->blockCount())
            .arg(canvas->connCount())
            .arg(canvas->lastRouteMs(), 0, 'f', 3)
            .arg(canvas->lastPaintMs(), 0, 'f', 3));
}
