#ifndef SOFTIONICS_GUI_BF16WINDOW_H
#define SOFTIONICS_GUI_BF16WINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QVector>
#include <QPointF>

#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPushButton>
#include <QLabel>

#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QLineSeries>

class BleWorker;

class BF16Window : public QMainWindow {
    Q_OBJECT
public:
    explicit BF16Window(BleWorker* worker, QWidget* parent = nullptr);
    ~BF16Window();

protected:
    void closeEvent(QCloseEvent* e) override;

private slots:
    void onPose(double x, double y, double z, double q1, double q2, double err, bool quiet, bool hasPose);
    void onTick();
    void onParamsChanged();
    void onResetSolver();
    void onClearPath();

private:
    void buildUi();
    void updateAxesAndDraw();

private:
    struct PosePkt {
        bool hasPose;
        bool quiet;
        double x, y, z, q1, q2, err;
    };

    BleWorker* worker_ = nullptr;

    QChartView* view_ = nullptr;
    QChart* chart_ = nullptr;
    QValueAxis* axX_ = nullptr;
    QValueAxis* axY_ = nullptr;

    QScatterSeries* sensors_ = nullptr;
    QLineSeries* path_ = nullptr;
    QScatterSeries* cur_ = nullptr;

    QLabel* lbStats_ = nullptr;

    QDoubleSpinBox* spR_ = nullptr;
    QDoubleSpinBox* spC_ = nullptr;
    QDoubleSpinBox* spAlpha_ = nullptr;
    QDoubleSpinBox* spQuiet_ = nullptr;
    QSpinBox* spMaxPoints_ = nullptr;

    QPushButton* btnReset_ = nullptr;
    QPushButton* btnClear_ = nullptr;

    QTimer* timer_ = nullptr;

    QVector<PosePkt> pending_;
    QVector<QPointF> pathBuf_;
    PosePkt last_{false,false,0,0,0,0,0,0};
};

#endif
