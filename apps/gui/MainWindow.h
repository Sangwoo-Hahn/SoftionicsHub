#pragma once
#include <QMainWindow>
#include <QThread>
#include <QTimer>
#include <QVector>
#include <QList>
#include <QPointF>

#include <QListWidget>
#include <QLabel>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLineEdit>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QChart>

#include <vector>

#include "BleWorker.h"
#include "hub/Pipeline.h"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void onScanUpdated(QVector<DeviceInfo> devices);
    void onStatus(QString text);
    void onConnected(QString name, QString addr);
    void onDisconnected();
    void onFrame(qulonglong t_ns, QVector<float> x, bool modelValid, float modelOut);
    void onStats(qulonglong ok, qulonglong bad);
    void onBiasState(bool hasBias, bool capturing);

    void onStreamStats(qulonglong totalSamples, double totalTimeSec, qulonglong last1sSamples, double lastDtSec);

    void onPoseReady(double x, double y, double z, double q1, double q2, double err, bool quiet, bool hasPose);

    void onDeviceClicked(QListWidgetItem* item);

    void onAnyControlChanged();
    void applyPipelineNow();

    void onBiasCapture();
    void onBiasSave();

    void onBrowseCsv();
    void onToggleRecord(bool on);

    void onPlotTick();

private:
    void buildUi();
    void rebuildTimePlot(int n_ch);
    hub::PipelineConfig readCfgFromUi() const;
    void updateDeviceListDecor();
    void clearTimePlotData();
    void updateXYPlot();

private:
    struct PosePkt {
        bool hasPose;
        bool quiet;
        double x;
        double y;
        double z;
        double q1;
        double q2;
        double err;
    };

private:
    QThread workerThread_;
    BleWorker* worker_ = nullptr;

    QVector<DeviceInfo> devices_;
    QString connectedAddr_;

    QListWidget* list_ = nullptr;
    QLabel* status_ = nullptr;
    QLabel* conn_ = nullptr;
    QLabel* stats_ = nullptr;

    QLabel* lb_stream_stats_ = nullptr;
    QLabel* lb_pose_stats_ = nullptr;

    QDoubleSpinBox* sp_xwin_ = nullptr;
    QDoubleSpinBox* sp_ycenter_ = nullptr;
    QDoubleSpinBox* sp_yabs_ = nullptr;
    QCheckBox* cb_yauto_ = nullptr;

    QCheckBox* cb_ma_ = nullptr;
    QSpinBox* sp_ma_ = nullptr;

    QCheckBox* cb_ema_ = nullptr;
    QDoubleSpinBox* sp_alpha_ = nullptr;

    QCheckBox* cb_notch_ = nullptr;
    QDoubleSpinBox* sp_fs_ = nullptr;
    QDoubleSpinBox* sp_f0_ = nullptr;
    QDoubleSpinBox* sp_q_ = nullptr;

    QCheckBox* cb_bias_apply_ = nullptr;
    QSpinBox* sp_bias_frames_ = nullptr;
    QPushButton* btn_bias_cap_ = nullptr;
    QPushButton* btn_bias_save_ = nullptr;
    QLabel* lb_bias_state_ = nullptr;

    QCheckBox* cb_record_ = nullptr;
    QLineEdit* ed_csv_path_ = nullptr;
    QPushButton* btn_browse_csv_ = nullptr;

    QChartView* timeView_ = nullptr;
    QChart* timeChart_ = nullptr;
    QValueAxis* timeAxX_ = nullptr;
    QValueAxis* timeAxY_ = nullptr;
    QLineSeries* timeCenterLine_ = nullptr;
    QVector<QLineSeries*> timeSeries_;
    QVector<QList<QPointF>> timeBuffers_;

    std::vector<QVector<float>> pending_samples_;
    double plotFsUsed_ = 0.0;
    unsigned long long plotSampleIndex_ = 0;

    QChartView* xyView_ = nullptr;
    QChart* xyChart_ = nullptr;
    QValueAxis* xyAxX_ = nullptr;
    QValueAxis* xyAxY_ = nullptr;
    QScatterSeries* xySensors_ = nullptr;
    QLineSeries* xyPath_ = nullptr;
    QScatterSeries* xyCurrent_ = nullptr;

    std::vector<PosePkt> pending_pose_;
    QList<QPointF> xyPathBuf_;
    int xyMaxPoints_ = 40;

    PosePkt lastPose_{false,false,0,0,0,0,0,0};

    QTimer* plotTimer_ = nullptr;
    QTimer* applyTimer_ = nullptr;

    bool fsAutoSetDone_ = false;
};
