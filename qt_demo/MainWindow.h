#pragma once

#include "CanvasWidget.h"
#include <QCheckBox>
#include <QComboBox>
#include <QMainWindow>

class MainWindow : public QMainWindow {
    Q_OBJECT
  public:
    explicit MainWindow(unsigned initialScale = 10000, QWidget *parent = nullptr);
    CanvasWidget *canvasWidget() const { return canvas; }

  private slots:
    void onScaleChanged(int index);
    void onModeChanged(int index);
    void onThirdPartyToggled(bool checked);
    void updateStatusSummary();

  private:
    CanvasWidget *canvas = nullptr;
    QComboBox *scaleCombo = nullptr;
    QComboBox *modeCombo = nullptr;
    QCheckBox *thirdPartyCheck = nullptr;
};
