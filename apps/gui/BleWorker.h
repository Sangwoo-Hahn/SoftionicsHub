#pragma once
#include <QObject>
#include <QVector>
#include <QString>
#include <QMutex>
#include <QMutexLocker>

#include <atomic>
#include <fstream>
#include <memory>
#include <thread>
#include <vector>
#include <optional>
#include <deque>
#include <cstdint>

#include <simpleble/SimpleBLE.h>

#include "hub/Framer.h"
#include "hub/Parser.h"
#include "hub/Pipeline.h"
#include "hub/model/Linear.h"

struct DeviceInfo {
    QString name;
    QString address;
    int rssi = 0;
};

class BleWorker : public QObject {
    Q_OBJECT
public:
    explicit BleWorker(QObject* parent = nullptr);
    ~BleWorker();

public slots:
    void startAuto(QString prefix);
    void connectToIndex(int index);
    void disconnectDevice();

    void setPipelineConfig(hub::PipelineConfig cfg);
    void startBiasCapture(int frames);

    void startCsv(QString path);
    void stopCsv();

    void loadWeights(QString path);

    // ---- Bias 저장 CSV ----
    void saveBiasCsv(QString path);

signals:
    void scanUpdated(QVector<DeviceInfo> devices);
    void statusText(QString text);
    void connected(QString name, QString address);
    void disconnected();

    void frameReady(qulonglong t_ns, QVector<float> x, bool modelValid, float modelOut);
    void statsUpdated(qulonglong ok, qulonglong bad);

    void biasStateChanged(bool hasBias, bool capturing);

    // totalSamples, totalTimeSec, last1sSamples, lastDtSec
    void streamStats(qulonglong totalSamples, double totalTimeSec, qulonglong last1sSamples, double lastDtSec);

private:
    void startScanning();
    void stopScanning();
    void scanLoop();
    void notifyStart();
    void notifyStop();

    void resetStreamStatsLocked();

private:
    std::atomic<bool> scanning_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> quitting_{false};
    std::thread scanThread_;

    QString prefix_{"Softionics"};

    std::optional<SimpleBLE::Adapter> adapter_;
    std::vector<SimpleBLE::Peripheral> peripherals_;
    QMutex periphMu_;

    std::optional<SimpleBLE::Peripheral> active_;
    std::optional<SimpleBLE::BluetoothUUID> svc_;
    std::optional<SimpleBLE::BluetoothUUID> chr_;

    hub::LineFramer framer_;
    hub::CsvFloatParser parser_;

    hub::Pipeline pipe_;
    hub::PipelineConfig cfg_;
    QMutex pipeMu_;

    bool lastBiasHas_ = false;
    bool lastBiasCapturing_ = false;

    std::atomic<size_t> n_ch_{0};

    std::atomic<qulonglong> ok_{0};
    std::atomic<qulonglong> bad_{0};

    std::atomic<bool> csvOn_{false};
    QString csvPath_;
    std::unique_ptr<std::ofstream> csv_;
    bool csvHeaderWritten_ = false;
    uint64_t csv_t0_ns_ = 0;
    QMutex csvMu_;

    std::atomic<bool> weightsPending_{false};
    std::vector<float> weights_;

    // ---- stream stat (pipeMu_ 아래에서만 접근/수정) ----
    uint64_t st_first_ns_ = 0;
    uint64_t st_prev_ns_ = 0;
    uint64_t st_last_ns_ = 0;
    uint64_t st_last_dt_ns_ = 0;
    uint64_t st_total_samples_ = 0;
    uint64_t st_last_emit_ns_ = 0;
    std::deque<uint64_t> st_last1s_ts_;
};
