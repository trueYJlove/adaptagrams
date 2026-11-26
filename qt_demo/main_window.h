#ifndef LOCALROUTE_QT_DEMO_MAIN_WINDOW_H_
#define LOCALROUTE_QT_DEMO_MAIN_WINDOW_H_

#include <QCheckBox>
#include <QComboBox>
#include <QMainWindow>

#include "canvas_widget.h"

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
  void UpdateStatusSummary();

 private:
  CanvasWidget* canvas_ = nullptr;
  QComboBox* scale_combo_ = nullptr;
  QComboBox* mode_combo_ = nullptr;
  QCheckBox* third_party_check_ = nullptr;
};

#endif  // LOCALROUTE_QT_DEMO_MAIN_WINDOW_H_
