#include "main_window.h"

#include <QDockWidget>
#include <QFormLayout>
#include <QGroupBox>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>

MainWindow::MainWindow(unsigned initial_scale, QWidget* parent)
    : QMainWindow(parent) {
  setWindowTitle("Local Orthogonal Router - 万级正交布线与自由建模演示");
  resize(1480, 900);

  canvas_ = new CanvasWidget(this);
  setCentralWidget(canvas_);

  QToolBar* toolbar = addToolBar("Controls");
  toolbar->setMovable(false);

  toolbar->addWidget(new QLabel(" 拓扑规模: ", toolbar));
  scale_combo_ = new QComboBox(toolbar);
  scale_combo_->addItem("0 模块 (空白画布)", 0);
  scale_combo_->addItem("1,000 模块 (2k 线)", 1000);
  scale_combo_->addItem("3,000 模块 (6k 线)", 3000);
  scale_combo_->addItem("5,000 模块 (10k 线)", 5000);
  scale_combo_->addItem("10,000 模块 (20k 线)", 10000);
  scale_combo_->addItem("预设: B1-B2 10线平行对连", -1);
  scale_combo_->addItem("预设: B9/B3/B8 级联布线", -2);
  const int init_idx = scale_combo_->findData(static_cast<int>(initial_scale));
  scale_combo_->setCurrentIndex(init_idx >= 0 ? init_idx : 4);
  toolbar->addWidget(scale_combo_);

  auto* clear_btn = new QPushButton("清空画布", toolbar);
  toolbar->addWidget(clear_btn);

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

  nudging_check_ = new QCheckBox("通道分轨 (Nudging)", toolbar);
  nudging_check_->setChecked(true);
  toolbar->addWidget(nudging_check_);

  highlight_fallback_check_ = new QCheckBox("高亮兜底线 (Debug)", toolbar);
  highlight_fallback_check_->setChecked(false);
  toolbar->addWidget(highlight_fallback_check_);

  toolbar->addSeparator();
  auto* fit_btn = new QPushButton("适应全图 (F)", toolbar);
  auto* reset_btn = new QPushButton("局部 100% 视角 (R)", toolbar);
  auto* route_all_btn = new QPushButton("重新全图布线", toolbar);
  toolbar->addWidget(fit_btn);
  toolbar->addWidget(reset_btn);
  toolbar->addWidget(route_all_btn);

  BuildDocks();

  connect(scale_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &MainWindow::OnScaleChanged);
  connect(clear_btn, &QPushButton::clicked, this, &MainWindow::OnClearCanvas);
  connect(mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &MainWindow::OnModeChanged);
  connect(third_party_check_, &QCheckBox::toggled, this,
          &MainWindow::OnThirdPartyToggled);
  connect(nudging_check_, &QCheckBox::toggled, this,
          &MainWindow::OnChannelNudgingToggled);
  connect(highlight_fallback_check_, &QCheckBox::toggled, this,
          &MainWindow::OnHighlightFallbackToggled);
  connect(fit_btn, &QPushButton::clicked, canvas_, &CanvasWidget::FitAll);
  connect(reset_btn, &QPushButton::clicked, canvas_,
          &CanvasWidget::ResetLocalView);
  connect(route_all_btn, &QPushButton::clicked, canvas_,
          &CanvasWidget::RouteAll);
  connect(canvas_, &CanvasWidget::StatusChanged, this,
          &MainWindow::UpdateStatusSummary);
  connect(canvas_, &CanvasWidget::SelectionChanged, this,
          &MainWindow::SyncInspectorFromSelection);

  canvas_->LoadTopology(scale_combo_->currentData().toUInt());
  SyncInspectorFromSelection();
  UpdateStatusSummary();
}

void MainWindow::BuildDocks() {
  auto* lib_dock = new QDockWidget("图元库 (拖拽到画布)", this);
  lib_dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  lib_dock->setFeatures(QDockWidget::DockWidgetMovable |
                        QDockWidget::DockWidgetFloatable);
  library_ = new LibraryWidget(lib_dock);
  lib_dock->setWidget(library_);
  lib_dock->setMinimumWidth(220);
  addDockWidget(Qt::LeftDockWidgetArea, lib_dock);
  connect(library_, &LibraryWidget::TemplateActivated, canvas_,
          &CanvasWidget::SpawnBlockAtCenter);

  auto* insp_dock = new QDockWidget("模块端口与编辑", this);
  insp_dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  insp_dock->setFeatures(QDockWidget::DockWidgetMovable |
                         QDockWidget::DockWidgetFloatable);
  insp_dock->setMinimumWidth(240);

  auto* panel = new QWidget(insp_dock);
  auto* vbox = new QVBoxLayout(panel);

  sel_title_label_ = new QLabel("未选中模块", panel);
  sel_title_label_->setWordWrap(true);
  sel_title_label_->setStyleSheet("font-weight: bold; padding: 4px 0;");
  vbox->addWidget(sel_title_label_);

  auto* port_box = new QGroupBox("自定义端口个数", panel);
  auto* form = new QFormLayout(port_box);
  auto make_spin = [&](int max_val) {
    auto* sp = new QSpinBox(port_box);
    sp->setRange(0, max_val);
    return sp;
  };
  left_spin_ = make_spin(16);
  right_spin_ = make_spin(16);
  top_spin_ = make_spin(8);
  bottom_spin_ = make_spin(8);
  form->addRow("左端口 (输入):", left_spin_);
  form->addRow("右端口 (输出):", right_spin_);
  form->addRow("上端口:", top_spin_);
  form->addRow("下端口:", bottom_spin_);
  vbox->addWidget(port_box);

  make_obs_btn_ = new QPushButton("一键设为障碍物 (0端口)", panel);
  clear_wires_btn_ = new QPushButton("清除该模块所有连线", panel);
  delete_sel_btn_ = new QPushButton("删除选中项 (Del)", panel);
  vbox->addWidget(make_obs_btn_);
  vbox->addWidget(clear_wires_btn_);
  vbox->addWidget(delete_sel_btn_);

  auto* tips = new QLabel(
      "操作提示:\n"
      "• 从左侧图元库拖拽模块到画布（或双击）\n"
      "• 鼠标悬停任意端口圆点（变绿）按住拖到另一端口完成连线\n"
      "• 选中模块后可在此修改各方向端口数量\n"
      "• 右键模块/连线可快速增删端口或删除\n"
      "• 点“清空画布”可从空白开始搭避障测试场景",
      panel);
  tips->setWordWrap(true);
  tips->setStyleSheet("color: #4a5568; font-size: 12px; padding-top: 8px;");
  vbox->addWidget(tips);
  vbox->addStretch(1);

  insp_dock->setWidget(panel);
  addDockWidget(Qt::RightDockWidgetArea, insp_dock);

  for (QSpinBox* sp : {left_spin_, right_spin_, top_spin_, bottom_spin_}) {
    connect(sp, QOverload<int>::of(&QSpinBox::valueChanged), this,
            &MainWindow::OnPortSpinChanged);
  }
  connect(make_obs_btn_, &QPushButton::clicked, this, [this]() {
    canvas_->SetSelectedBlockPorts(0, 0, 0, 0);
    SyncInspectorFromSelection();
  });
  connect(clear_wires_btn_, &QPushButton::clicked, this, [this]() {
    localroute::Id bid = 0;
    if (canvas_->GetSelectedBlockPorts(&bid, nullptr, nullptr, nullptr,
                                       nullptr)) {
      canvas_->ClearBlockWires(bid);
    }
  });
  connect(delete_sel_btn_, &QPushButton::clicked, canvas_,
          &CanvasWidget::DeleteSelection);
}

void MainWindow::OnScaleChanged(int index) {
  const int val = scale_combo_->itemData(index).toInt();
  if (val == -1) {
    canvas_->LoadPresetParallelBundle();
  } else if (val == -2) {
    canvas_->LoadPresetCascade();
  } else {
    canvas_->LoadTopology(static_cast<unsigned>(val));
  }
}

void MainWindow::OnClearCanvas() {
  const QSignalBlocker blocker(scale_combo_);
  scale_combo_->setCurrentIndex(0);
  canvas_->LoadTopology(0);
}

void MainWindow::OnModeChanged(int index) {
  const auto m = static_cast<CanvasWidget::RouteMode>(
      mode_combo_->itemData(index).toInt());
  canvas_->SetRouteMode(m);
}

void MainWindow::OnThirdPartyToggled(bool checked) {
  canvas_->SetRerouteThirdPartyBlocked(checked);
}

void MainWindow::OnChannelNudgingToggled(bool checked) {
  canvas_->SetEnableChannelNudging(checked);
}

void MainWindow::OnHighlightFallbackToggled(bool checked) {
  canvas_->SetHighlightFallback(checked);
}

void MainWindow::OnPortSpinChanged() {
  if (updating_inspector_) {
    return;
  }
  canvas_->SetSelectedBlockPorts(left_spin_->value(), right_spin_->value(),
                                 top_spin_->value(), bottom_spin_->value());
}

void MainWindow::SyncInspectorFromSelection() {
  updating_inspector_ = true;
  localroute::Id bid = 0;
  int l = 0;
  int r = 0;
  int t = 0;
  int b = 0;
  const bool single = canvas_->GetSelectedBlockPorts(&bid, &l, &r, &t, &b);

  for (QSpinBox* sp : {left_spin_, right_spin_, top_spin_, bottom_spin_}) {
    sp->setEnabled(single);
  }
  make_obs_btn_->setEnabled(single);
  clear_wires_btn_->setEnabled(single);
  delete_sel_btn_->setEnabled(canvas_->selected_block_count() > 0 ||
                              canvas_->has_selected_conn());

  if (single) {
    sel_title_label_->setText(QString("已选中: Block #%1").arg(bid));
    left_spin_->setValue(l);
    right_spin_->setValue(r);
    top_spin_->setValue(t);
    bottom_spin_->setValue(b);
  } else if (canvas_->selected_block_count() > 1) {
    sel_title_label_->setText(
        QString("已框选 %1 个模块").arg(canvas_->selected_block_count()));
  } else if (canvas_->has_selected_conn()) {
    sel_title_label_->setText("已选中 1 条连线 (可按 Del 删除)");
  } else {
    sel_title_label_->setText("未选中模块（请点击画布模块或从左侧拖入）");
  }
  updating_inspector_ = false;
}

void MainWindow::UpdateStatusSummary() {
  statusBar()->showMessage(
      QString("滚轮缩放 | 中键或空格+左键平移 | 悬停端口拖线 | Del 删除 | F "
              "全图 / R 局部 | %1 模块 / %2 连线 | Route %3 ms | Paint %4 ms")
          .arg(canvas_->block_count())
          .arg(canvas_->conn_count())
          .arg(canvas_->last_route_ms(), 0, 'f', 3)
          .arg(canvas_->last_paint_ms(), 0, 'f', 3));
}
