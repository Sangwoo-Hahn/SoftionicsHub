#ifndef SOFTIONICS_GUI_POSITIONTRACKINGWINDOW_H
#define SOFTIONICS_GUI_POSITIONTRACKINGWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QTimer>
#include <QVector>
#include <QPointF>
#include <QComboBox>
#include <QFormLayout>
#include <QScrollArea>
#include <QPushButton>
#include <QSpinBox>
#include <QLabel>
#include <QShowEvent>
#include <QHideEvent>
#include <QCloseEvent>

#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QLineSeries>

#include "hub/model/PositionTrackingRegistry.h"

class BleWorker;
class PositionTrackingEngine;

class PositionTrackingWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit PositionTrackingWindow(BleWorker* worker, QWidget* parent = nullptr);
    ~PositionTrackingWindow();

protected:
    void showEvent(QShowEvent* e) override;
    void hideEvent(QHideEvent* e) override;
    void closeEvent(QCloseEvent* e) override;

private slots:
    void onAlgoChanged(int idx);
    void onApplyParams();
    void onResetAlgo();
    void onClearPath();

    void onEngineOut(double x, double y, double z, double confidence, double q1, double q2, double err, bool quiet, bool valid);
    void onEngineStatus(QString text);
    void onTick();

private:
    void buildUi();
    void rebuildParamUi(const hub::pt::AlgoInfo& info);
    QVector<double> collectParams() const;
    void updateAxesAndDraw();

private:
    struct OutPkt {
        bool valid;
        bool quiet;
        double x, y, z, confidence, q1, q2, err;
    };

    BleWorker* worker_ = nullptr;

    QThread engineThread_;
    PositionTrackingEngine* engine_ = nullptr;

    QTimer* timer_ = nullptr;
    QTimer* paramTimer_ = nullptr;

    QComboBox* cbAlgo_ = nullptr;
    QLabel* lbAlgoInfo_ = nullptr;

    QWidget* paramBox_ = nullptr;
    QFormLayout* paramForm_ = nullptr;
    QVector<QDoubleSpinBox*> paramSpins_;
    hub::pt::AlgoInfo curInfo_;

    QPushButton* btnApply_ = nullptr;
    QPushButton* btnReset_ = nullptr;
    QPushButton* btnClear_ = nullptr;
    QSpinBox* spPathLen_ = nullptr;

    QChartView* view_ = nullptr;
    QChart* chart_ = nullptr;
    QValueAxis* axX_ = nullptr;
    QValueAxis* axY_ = nullptr;
    QScatterSeries* sensors_ = nullptr;
    QLineSeries* path_ = nullptr;
    QScatterSeries* cur_ = nullptr;

    QLabel* lbStats_ = nullptr;

    QVector<OutPkt> pending_;
    QVector<QPointF> pathBuf_;
    OutPkt last_{false, false, 0, 0, 0, 0, 0, 0, 0};
    QString engineStatusText_;

    bool connected_ = false;
};

#endif
