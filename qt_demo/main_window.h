#ifndef LOCALROUTE_QT_DEMO_MAIN_WINDOW_H_
#define LOCALROUTE_QT_DEMO_MAIN_WINDOW_H_

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QSpinBox>

#include "canvas_widget.h"
#include "library_widget.h"

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(unsigned initial_scale = 10000,
                      QWidget* parent = nullptr);
  CanvasWidget* canvas_widget() const { return canvas_; }

 private slots:
  void OnScaleChanged(int index);
  void OnModeChanged(int index);
  void OnThirdPartyToggled(bool checked);
  void OnChannelNudgingToggled(bool checked);
  void OnHighlightFallbackToggled(bool checked);
  void OnClearCanvas();
  void OnPortSpinChanged();
  void SyncInspectorFromSelection();
  void UpdateStatusSummary();

 private:
  void BuildDocks();

  CanvasWidget* canvas_ = nullptr;
  LibraryWidget* library_ = nullptr;
  QComboBox* scale_combo_ = nullptr;
  QComboBox* mode_combo_ = nullptr;
  QCheckBox* third_party_check_ = nullptr;
  QCheckBox* nudging_check_ = nullptr;
  QCheckBox* highlight_fallback_check_ = nullptr;

  // Inspector controls
  QLabel* sel_title_label_ = nullptr;
  QSpinBox* left_spin_ = nullptr;
  QSpinBox* right_spin_ = nullptr;
  QSpinBox* top_spin_ = nullptr;
  QSpinBox* bottom_spin_ = nullptr;
  QPushButton* make_obs_btn_ = nullptr;
  QPushButton* clear_wires_btn_ = nullptr;
  QPushButton* delete_sel_btn_ = nullptr;
  bool updating_inspector_ = false;
};

#endif  // LOCALROUTE_QT_DEMO_MAIN_WINDOW_H_
