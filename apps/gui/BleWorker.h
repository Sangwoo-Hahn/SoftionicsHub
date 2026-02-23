#ifndef SOFTIONICS_GUI_BLEWORKER_H
#define SOFTIONICS_GUI_BLEWORKER_H

#include <QObject>
#include <QVector>
#include <QString>
#include <QMutex>
#include <QMutexLocker>

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <atomic>
#include <fstream>
#include <memory>
#include <thread>
#include <vector>
#include <optional>
#include <deque>
#include <cstdint>
#include <string_view>

#include <simpleble/SimpleBLE.h>

#include "hub/Framer.h"
#include "hub/Parser.h"
#include "hub/Pipeline.h"

enum class DeviceKind : int {
    Ble = 0,
    Serial = 1,
};

struct DeviceInfo {
    DeviceKind kind = DeviceKind::Ble;
    QString name;
    QString address;
    int rssi = 0; // BLE only
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

    void saveBiasCsv(QString path);

signals:
    void scanUpdated(QVector<DeviceInfo> devices);
    void statusText(QString text);
    void connected(QString name, QString address);
    void disconnected();

    void frameReady(qulonglong t_ns, QVector<float> x, bool modelValid, float modelOut);
    void statsUpdated(qulonglong ok, qulonglong bad);

    void biasStateChanged(bool hasBias, bool capturing);
    void streamStats(qulonglong totalSamples, double totalTimeSec, qulonglong last1sSamples, double lastDtSec);

private:
    void startScanning();
    void stopScanning();
    void scanLoop();
    void notifyStart();
    void notifyStop();

    void processChunk(std::string_view chunk);

    void serialConnect(const QString& portName);
    void serialDisconnect();

    void resetStreamStatsLocked();

private slots:
    void onSerialReadyRead();
    void onSerialError(QSerialPort::SerialPortError error);

private:
    std::atomic<bool> scanning_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> quitting_{false};
    std::thread scanThread_;

    QString prefix_{"Softionics"};

    std::optional<SimpleBLE::Adapter> adapter_;
    std::vector<SimpleBLE::Peripheral> peripherals_;
    QMutex periphMu_;

    QVector<DeviceInfo> lastScan_;
    QMutex scanMu_;

    std::optional<SimpleBLE::Peripheral> active_;
    std::optional<SimpleBLE::BluetoothUUID> svc_;
    std::optional<SimpleBLE::BluetoothUUID> chr_;

    std::atomic<int> linkType_{0}; // 0 none, 1 BLE, 2 Serial

    QSerialPort* serial_ = nullptr;
    QString serialPort_;


    // Serial streams can start mid-line when the port is opened.
    // We sync to the next newline before feeding data to the CSV framer/parser.
    std::atomic<bool> serialSynced_{false};

    std::atomic<uint64_t> stream_t0_ns_{0};

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

    uint64_t st_first_ns_ = 0;
    uint64_t st_prev_ns_ = 0;
    uint64_t st_last_ns_ = 0;
    uint64_t st_last_dt_ns_ = 0;
    uint64_t st_total_samples_ = 0;
    uint64_t st_last_emit_ns_ = 0;
    std::deque<uint64_t> st_last1s_ts_;
};

#endif
