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
#include <QtCharts/QValueAxis>
#include <QtCharts/QChart>

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

    void onDeviceDoubleClicked(QListWidgetItem*);

    void onAnyControlChanged();
    void applyPipelineNow();

    void onBiasCapture();
    void onBrowseCsv();
    void onToggleRecord(bool on);
    void onLoadWeights();

    void onPlotTick();

private:
    void buildUi();
    void rebuildPlot(int n_ch);

    hub::PipelineConfig readCfgFromUi() const;
    void updateDeviceListDecor();

private:
    QThread workerThread_;
    BleWorker* worker_ = nullptr;

    QVector<DeviceInfo> devices_;
    QString connectedAddr_;

    QListWidget* list_ = nullptr;
    QLabel* status_ = nullptr;
    QLabel* conn_ = nullptr;
    QLabel* stats_ = nullptr;

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

    QCheckBox* cb_bias_ = nullptr;
    QSpinBox* sp_bias_frames_ = nullptr;
    QPushButton* btn_bias_cap_ = nullptr;
    QLabel* biasInfo_ = nullptr;

    QCheckBox* cb_model_ = nullptr;
    QDoubleSpinBox* sp_model_bias_ = nullptr;
    QLabel* modelOut_ = nullptr;
    QPushButton* btn_load_weights_ = nullptr;

    QCheckBox* cb_record_ = nullptr;
    QLineEdit* ed_csv_path_ = nullptr;
    QPushButton* btn_browse_csv_ = nullptr;

    QChartView* chartView_ = nullptr;
    QChart* chart_ = nullptr;
    QValueAxis* axX_ = nullptr;
    QValueAxis* axY_ = nullptr;

    QVector<QLineSeries*> series_;
    QVector<QList<QPointF>> buffers_;

    QVector<float> lastFrame_;
    qulonglong lastTsNs_ = 0;
    qulonglong t0Ns_ = 0;

    QTimer* plotTimer_ = nullptr;
    QTimer* applyTimer_ = nullptr;
};
