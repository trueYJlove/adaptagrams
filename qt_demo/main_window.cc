#include "main_window.h"

#include <QLabel>
#include <QPushButton>
#include <QStatusBar>
#include <QToolBar>

MainWindow::MainWindow(unsigned initial_scale, QWidget* parent)
    : QMainWindow(parent) {
  setWindowTitle("Local Orthogonal Router - 万级正交布线实时交互演示");
  resize(1360, 860);

  canvas_ = new CanvasWidget(this);
  setCentralWidget(canvas_);

  QToolBar* toolbar = addToolBar("Controls");
  toolbar->setMovable(false);

  toolbar->addWidget(new QLabel(" 拓扑规模: ", toolbar));
  scale_combo_ = new QComboBox(toolbar);
  scale_combo_->addItem("1,000 模块 (2k 线)", 1000);
  scale_combo_->addItem("3,000 模块 (6k 线)", 3000);
  scale_combo_->addItem("5,000 模块 (10k 线)", 5000);
  scale_combo_->addItem("10,000 模块 (20k 线)", 10000);
  const int init_idx = scale_combo_->findData(initial_scale);
  scale_combo_->setCurrentIndex(init_idx >= 0 ? init_idx : 3);
  toolbar->addWidget(scale_combo_);

  toolbar->addSeparator();
  toolbar->addWidget(new QLabel(" 路由模式: ", toolbar));
  mode_combo_ = new QComboBox(toolbar);
  mode_combo_->addItem("Async (异步)",
                       static_cast<int>(CanvasWidget::RouteMode::kAsync));
  mode_combo_->addItem("Sync (同步)",
                       static_cast<int>(CanvasWidget::RouteMode::kSync));
  toolbar->addWidget(mode_combo_);

  toolbar->addSeparator();
  third_party_check_ = new QCheckBox("第三方压线重算 (默认关闭)", toolbar);
  third_party_check_->setChecked(false);
  toolbar->addWidget(third_party_check_);

  toolbar->addSeparator();
  auto* fit_btn = new QPushButton("适应全图 (F)", toolbar);
  auto* reset_btn = new QPushButton("局部 100% 视角 (R)", toolbar);
  auto* route_all_btn = new QPushButton("重新全图布线", toolbar);
  toolbar->addWidget(fit_btn);
  toolbar->addWidget(reset_btn);
  toolbar->addWidget(route_all_btn);

  connect(scale_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &MainWindow::OnScaleChanged);
  connect(mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &MainWindow::OnModeChanged);
  connect(third_party_check_, &QCheckBox::toggled, this,
          &MainWindow::OnThirdPartyToggled);
  connect(fit_btn, &QPushButton::clicked, canvas_, &CanvasWidget::FitAll);
  connect(reset_btn, &QPushButton::clicked, canvas_,
          &CanvasWidget::ResetLocalView);
  connect(route_all_btn, &QPushButton::clicked, canvas_,
          &CanvasWidget::RouteAll);
  connect(canvas_, &CanvasWidget::StatusChanged, this,
          &MainWindow::UpdateStatusSummary);

  canvas_->LoadTopology(scale_combo_->currentData().toUInt());
  UpdateStatusSummary();
}

void MainWindow::OnScaleChanged(int index) {
  const unsigned n = scale_combo_->itemData(index).toUInt();
  canvas_->LoadTopology(n);
}

void MainWindow::OnModeChanged(int index) {
  const auto m = static_cast<CanvasWidget::RouteMode>(
      mode_combo_->itemData(index).toInt());
  canvas_->SetRouteMode(m);
}

void MainWindow::OnThirdPartyToggled(bool checked) {
  canvas_->SetRerouteThirdPartyBlocked(checked);
}

void MainWindow::UpdateStatusSummary() {
  statusBar()->showMessage(
      QString(
          "滚轮缩放 | 中键或空格+左键平移 | 左键拖拽模块或框选 | F 全图 / R "
          "局部100% | 当前: %1 模块 / %2 连线 | Route %3 ms | Paint %4 ms")
          .arg(canvas_->block_count())
          .arg(canvas_->conn_count())
          .arg(canvas_->last_route_ms(), 0, 'f', 3)
          .arg(canvas_->last_paint_ms(), 0, 'f', 3));
}
