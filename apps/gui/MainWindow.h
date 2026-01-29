#ifndef SOFTIONICS_GUI_MAINWINDOW_H
#define SOFTIONICS_GUI_MAINWINDOW_H

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
#include <QtCharts/QValueAxis>
#include <QtCharts/QChart>

#include <vector>
#include <cstdint>

#include "BleWorker.h"
#include "hub/Pipeline.h"

class PositionTrackingWindow;

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

    void onDeviceClicked(QListWidgetItem* item);

    void onAnyControlChanged();
    void applyPipelineNow();

    void onBiasCapture();
    void onBiasSave();

    void onBrowseCsv();
    void onToggleRecord(bool on);

    void onOpenPositionTracking();
    void onPlotTick();

private:
    struct PendingFrame {
        double t;        // seconds on plot axis (uniform step)
        QVector<float> x;
    };

    void buildUi();
    void rebuildPlot(int n_ch);
    void clearPlotData();
    void updateDeviceListDecor();
    hub::PipelineConfig readCfgFromUi() const;

    void beginConnecting(const QString& addr, const QString& name);
    void endConnecting();

    void rescalePlotTime(double ratio); // scale existing x coordinates

private:
    QThread workerThread_;
    BleWorker* worker_ = nullptr;

    PositionTrackingWindow* ptWin_ = nullptr;

    QVector<DeviceInfo> devices_;

    QString connectedAddr_;
    QString connectedName_;

    QListWidget* list_ = nullptr;
    QLabel* status_ = nullptr;
    QLabel* conn_ = nullptr;
    QLabel* stats_ = nullptr;

    QLabel* lb_stream_stats_ = nullptr;

    // Plot controls
    QDoubleSpinBox* sp_xwin_ = nullptr;
    QDoubleSpinBox* sp_ycenter_ = nullptr;
    QDoubleSpinBox* sp_yabs_ = nullptr;
    QCheckBox* cb_yauto_ = nullptr;

    // Filters
    QCheckBox* cb_ma_ = nullptr;
    QSpinBox*  sp_ma_ = nullptr;

    QCheckBox* cb_ema_ = nullptr;
    QDoubleSpinBox* sp_alpha_ = nullptr;

    QCheckBox* cb_notch_ = nullptr;
    QDoubleSpinBox* sp_fs_ = nullptr;
    QDoubleSpinBox* sp_f0_ = nullptr;
    QDoubleSpinBox* sp_q_  = nullptr;

    // Bias
    QCheckBox* cb_bias_apply_ = nullptr;
    QSpinBox* sp_bias_frames_ = nullptr;
    QPushButton* btn_bias_cap_ = nullptr;
    QPushButton* btn_bias_save_ = nullptr;
    QLabel* lb_bias_state_ = nullptr;

    // PositionTracking
    QPushButton* btn_pt_ = nullptr;

    // CSV record
    QCheckBox* cb_record_ = nullptr;
    QLineEdit* ed_csv_path_ = nullptr;
    QPushButton* btn_browse_csv_ = nullptr;

    // Chart
    QChartView* chartView_ = nullptr;
    QChart* chart_ = nullptr;
    QValueAxis* axX_ = nullptr;
    QValueAxis* axY_ = nullptr;
    QLineSeries* centerLine_ = nullptr;

    QVector<QLineSeries*> series_;
    QVector<QList<QPointF>> buffers_;
    std::vector<PendingFrame> pending_;

    // ---- uniform-x plot clock ----
    uint64_t sampleIndex_ = 0;      // increments by 1 per sample(line)
    double plotFs_ = 200.0;         // fixed (can be updated once by auto-measure with rescale)
    double dtPlot_ = 1.0 / 200.0;   // 1/plotFs_

    QTimer* plotTimer_ = nullptr;
    QTimer* applyTimer_ = nullptr;

    bool fsAutoSetDone_ = false;

    // Connecting UI
    bool connecting_ = false;
    QString connectingAddr_;
    QString connectingName_;
    QTimer* connectTimeout_ = nullptr;
};

#endif
