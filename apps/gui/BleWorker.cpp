#include "BleWorker.h"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <QByteArray>
#include <QMetaObject>

static inline uint64_t now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

static inline bool starts_with(const std::string& s, const std::string& pfx) {
    return s.size() >= pfx.size() && std::memcmp(s.data(), pfx.data(), pfx.size()) == 0;
}

static std::optional<std::pair<SimpleBLE::BluetoothUUID, SimpleBLE::BluetoothUUID>>
pick_first_notify_char(SimpleBLE::Peripheral& p) {
    auto services = p.services();
    for (auto& s : services) {
        auto chars = s.characteristics();
        for (auto& c : chars) {
            if (c.can_notify()) return std::make_pair(s.uuid(), c.uuid());
        }
    }
    return std::nullopt;
}

BleWorker::BleWorker(QObject* parent) : QObject(parent) {
    if (SimpleBLE::Adapter::bluetooth_enabled()) {
        auto ads = SimpleBLE::Adapter::get_adapters();
        if (!ads.empty()) adapter_ = ads[0];
    }
}

BleWorker::~BleWorker() {
    quitting_.store(true);
    stopScanning();
    disconnectDevice();
}

void BleWorker::resetStreamStatsLocked() {
    st_first_ns_ = 0;
    st_prev_ns_ = 0;
    st_last_ns_ = 0;
    st_last_dt_ns_ = 0;
    st_total_samples_ = 0;
    st_last_emit_ns_ = 0;
    st_last1s_ts_.clear();
}

void BleWorker::startAuto(QString prefix) {
    prefix_ = prefix;

    if (!adapter_) {
        emit statusText("No Bluetooth adapter (Serial only)");
    } else {
        emit statusText("Scanning...");
    }

    startScanning();
}

void BleWorker::startScanning() {
    if (scanning_.exchange(true)) return;
    if (scanThread_.joinable()) scanThread_.join();
    scanThread_ = std::thread([this]() { this->scanLoop(); });
}

void BleWorker::stopScanning() {
    scanning_.store(false);
    if (scanThread_.joinable()) scanThread_.join();
}

void BleWorker::scanLoop() {
    while (scanning_.load() && !quitting_.load()) {
        QVector<DeviceInfo> list;

        // --- BLE scan (optional) ---
        if (adapter_) {
            try {
                adapter_->scan_for(1000);
                auto results = adapter_->scan_get_results();

                QMutexLocker lk(&periphMu_);
                peripherals_.clear();
                peripherals_.reserve(results.size());

                for (auto& p : results) {
                    auto name = p.identifier();
                    if (!starts_with(name, prefix_.toStdString())) continue;

                    peripherals_.push_back(p);

                    DeviceInfo d;
                    d.kind = DeviceKind::Ble;
                    d.name = QString::fromStdString(p.identifier());
                    d.address = QString::fromStdString(p.address());
                    d.rssi = p.rssi();
                    list.push_back(d);
                }
            } catch (...) {
                emit statusText("Scan error");
                QMutexLocker lk(&periphMu_);
                peripherals_.clear();
            }
        } else {
            QMutexLocker lk(&periphMu_);
            peripherals_.clear();
        }

        // --- Serial ports (COM) ---
        try {
            const auto ports = QSerialPortInfo::availablePorts();
            for (const auto& pi : ports) {
                QString port = pi.portName();
                if (port.isEmpty()) continue;

                QString friendly = port;

                QString desc = pi.description();
                QString manu = pi.manufacturer();

                if (!desc.isEmpty()) friendly += " - " + desc;
                if (!manu.isEmpty() && (desc.isEmpty() || !desc.contains(manu))) friendly += " (" + manu + ")";

                DeviceInfo d;
                d.kind = DeviceKind::Serial;
                d.name = friendly;
                d.address = port; // unique id / connect key
                d.rssi = 0;
                list.push_back(d);
            }
        } catch (...) {
            // ignore
        }

        {
            QMutexLocker lk(&scanMu_);
            lastScan_ = list;
        }

        emit scanUpdated(list);

        std::this_thread::sleep_for(std::chrono::milliseconds(120));
    }
}

void BleWorker::serialConnect(const QString& portName) {
    if (portName.isEmpty()) {
        emit statusText("Connect failed: Serial port empty");
        return;
    }

    if (!serial_) {
        serial_ = new QSerialPort(this);
        connect(serial_, &QSerialPort::readyRead, this, &BleWorker::onSerialReadyRead);
        connect(serial_, &QSerialPort::errorOccurred, this, &BleWorker::onSerialError);
    } else {
        if (serial_->isOpen()) serial_->close();
    }

    serialPort_ = portName;

    serial_->setPortName(portName);
    serial_->setBaudRate(QSerialPort::Baud115200);
    serial_->setDataBits(QSerialPort::Data8);
    serial_->setParity(QSerialPort::NoParity);
    serial_->setStopBits(QSerialPort::OneStop);
    serial_->setFlowControl(QSerialPort::NoFlowControl);

    if (!serial_->open(QIODevice::ReadOnly)) {
        emit statusText(QString("Connect failed: Serial open: %1").arg(serial_->errorString()));
        serialPort_.clear();
        return;
    }

    serialSynced_.store(false);
    framer_.clear();
    stream_t0_ns_.store(now_ns());

    {
        QMutexLocker lk(&pipeMu_);
        pipe_.reset();
        pipe_.set_config(cfg_);
        lastBiasHas_ = pipe_.bias_has();
        lastBiasCapturing_ = pipe_.bias_capturing();
        resetStreamStatsLocked();
    }

    emit biasStateChanged(lastBiasHas_, lastBiasCapturing_);
    emit streamStats(0, 0.0, 0, 0.0);

    n_ch_.store(0);
    ok_.store(0);
    bad_.store(0);

    linkType_.store(2);
    connected_.store(true);
}

void BleWorker::serialDisconnect() {
    if (serial_) {
        if (serial_->isOpen()) serial_->close();
    }
    serialPort_.clear();
    serialSynced_.store(false);
}

void BleWorker::connectToIndex(int index) {
    bool wasScanning = scanning_.load();
    stopScanning();

    DeviceInfo target;
    {
        QMutexLocker lk(&scanMu_);
        if (index < 0 || index >= lastScan_.size()) {
            if (wasScanning) startScanning();
            return;
        }
        target = lastScan_[index];
    }

    // --- Serial ---
    if (target.kind == DeviceKind::Serial) {
        // ✅ 같은 장치 재클릭으로 disconnect 토글하지 않음 (아무 것도 안 함)
        if (connected_.load() && linkType_.load() == 2 && serial_ && serial_->isOpen() && serialPort_ == target.address) {
            emit statusText("Already connected");
            if (wasScanning) startScanning();
            return;
        }

        if (connected_.load()) disconnectDevice();

        emit statusText("Connecting...");
        serialConnect(target.address);

        if (connected_.load() && linkType_.load() == 2) {
            emit connected(target.name, target.address);
            emit statusText("Connected");
        }

        if (wasScanning) startScanning();
        return;
    }

    // --- BLE ---
    if (!adapter_) {
        emit statusText("No Bluetooth adapter");
        if (wasScanning) startScanning();
        return;
    }

    SimpleBLE::Peripheral p;
    {
        QMutexLocker lk(&periphMu_);
        if (index < 0 || index >= (int)peripherals_.size()) {
            if (wasScanning) startScanning();
            return;
        }
        p = peripherals_[(size_t)index];
    }

    try {
        // ✅ 같은 장치 재클릭으로 disconnect 토글하지 않음 (아무 것도 안 함)
        if (connected_.load() && linkType_.load() == 1 && active_ && active_->address() == p.address()) {
            emit statusText("Already connected");
            if (wasScanning) startScanning();
            return;
        }

        // 다른 장치면 기존 연결은 끊고 새로 연결
        if (connected_.load()) disconnectDevice();

        emit statusText("Connecting...");
        p.connect();

        auto pair = pick_first_notify_char(p);
        if (!pair) {
            p.disconnect();
            emit statusText("No notify characteristic");
            if (wasScanning) startScanning();
            return;
        }

        active_ = p;
        svc_ = pair->first;
        chr_ = pair->second;

        // ✅ 물리적 끊김 자동 감지
        try {
            active_->set_callback_on_disconnected([this]() {
                if (quitting_.load()) return;
                QMetaObject::invokeMethod(this, [this]() {
                    if (quitting_.load()) return;
                    disconnectDevice();
                }, Qt::QueuedConnection);
            });
        } catch (...) {}

        framer_.clear();
        stream_t0_ns_.store(now_ns());

        {
            QMutexLocker lk(&pipeMu_);
            pipe_.reset();
            pipe_.set_config(cfg_);
            lastBiasHas_ = pipe_.bias_has();
            lastBiasCapturing_ = pipe_.bias_capturing();
            resetStreamStatsLocked();
        }

        emit biasStateChanged(lastBiasHas_, lastBiasCapturing_);
        emit streamStats(0, 0.0, 0, 0.0);

        n_ch_.store(0);
        ok_.store(0);
        bad_.store(0);

        linkType_.store(1);
        connected_.store(true);

        notifyStart();

        emit connected(QString::fromStdString(p.identifier()), QString::fromStdString(p.address()));
        emit statusText("Connected");
    } catch (...) {
        emit statusText("Connect failed");
    }

    // 스캔은 항상 유지
    if (wasScanning) startScanning();
}

void BleWorker::disconnectDevice() {
    // 중복/재진입 방지
    if (!connected_.exchange(false)) return;

    int lt = linkType_.exchange(0);

    if (lt == 1) {
        try { notifyStop(); } catch (...) {}
        try { if (active_) active_->disconnect(); } catch (...) {}

        active_.reset();
        svc_.reset();
        chr_.reset();
    } else if (lt == 2) {
        serialDisconnect();
    }

    stream_t0_ns_.store(0);
    framer_.clear();

    stopCsv();

    {
        QMutexLocker lk(&pipeMu_);
        lastBiasHas_ = false;
        lastBiasCapturing_ = false;
        resetStreamStatsLocked();
    }

    emit biasStateChanged(false, false);
    emit streamStats(0, 0.0, 0, 0.0);

    emit disconnected();
    emit statusText("Scanning...");

    // 스캔이 꺼져있을 수도 있으니 보장
    if (!scanning_.load() && !quitting_.load()) startScanning();
}

void BleWorker::notifyStart() {
    if (!active_ || !svc_ || !chr_) return;

    active_->notify(*svc_, *chr_, [this](SimpleBLE::ByteArray payload) {
        if (!connected_.load()) return;
        if (linkType_.load() != 1) return;

        std::string chunk(payload.begin(), payload.end());
        this->processChunk(std::string_view(chunk));
    });
}

void BleWorker::notifyStop() {
    if (!active_ || !svc_ || !chr_) return;
    active_->unsubscribe(*svc_, *chr_);
}

void BleWorker::onSerialReadyRead() {
    if (!connected_.load()) return;
    if (linkType_.load() != 2) return;
    if (!serial_) return;

    QByteArray data = serial_->readAll();
    if (data.isEmpty()) return;

    // Serial ports often start streaming mid-line when you open the port.
    // If we feed that first partial line into the CSV parser, it can lock
    // the channel count incorrectly (e.g., 10 instead of 16) and the plot
    // may stay empty afterwards. To avoid this, discard everything until
    // the next newline once per connect.
    if (!serialSynced_.load()) {
        int i_n = data.indexOf('\n');
        int i_r = data.indexOf('\r');

        int i = -1;
        if (i_n >= 0 && i_r >= 0) i = (i_n < i_r) ? i_n : i_r;
        else if (i_n >= 0) i = i_n;
        else i = i_r;

        if (i < 0) {
            // Still syncing: drop this chunk.
            return;
        }

        int adv = 1;
        if (data.at(i) == '\r' && (i + 1) < data.size() && data.at(i + 1) == '\n') adv = 2;

        data = data.mid(i + adv);
        serialSynced_.store(true);

        if (data.isEmpty()) return;
    }

    std::string chunk(data.constData(), (size_t)data.size());
    processChunk(std::string_view(chunk));
}

void BleWorker::onSerialError(QSerialPort::SerialPortError error) {
    if (error == QSerialPort::NoError) return;
    if (!connected_.load()) return;
    if (linkType_.load() != 2) return;
    if (quitting_.load()) return;
    if (!serial_) return;

    // unplug / fatal errors
    if (error == QSerialPort::ResourceError ||
        error == QSerialPort::DeviceNotFoundError ||
        error == QSerialPort::PermissionError ||
        error == QSerialPort::OpenError) {

        emit statusText(QString("Serial error: %1").arg(serial_->errorString()));

        QMetaObject::invokeMethod(this, [this]() {
            if (quitting_.load()) return;
            disconnectDevice();
        }, Qt::QueuedConnection);
    }
}

void BleWorker::processChunk(std::string_view chunk) {
    if (!connected_.load()) return;

    uint64_t stream_t0 = stream_t0_ns_.load();
    if (stream_t0 == 0) stream_t0 = now_ns();

    auto lines = framer_.push(chunk);

    for (auto& line : lines) {
        auto v = parser_.parse_line(line);
        if (!v) { bad_.fetch_add(1); continue; }

        size_t n = v->size();

        if (n_ch_.load() == 0) {
            n_ch_.store(n);
            QMutexLocker lk(&pipeMu_);
            pipe_.ensure_initialized(n);
            lastBiasHas_ = pipe_.bias_has();
            lastBiasCapturing_ = pipe_.bias_capturing();
            resetStreamStatsLocked();
            emit biasStateChanged(lastBiasHas_, lastBiasCapturing_);
            emit streamStats(0, 0.0, 0, 0.0);
        }

        if (n_ch_.load() != n) { bad_.fetch_add(1); continue; }

        uint64_t t = now_ns();

        hub::PipelineOut out;
        bool cap = false;
        bool has = false;
        bool emitBias = false;
        bool emitStream = false;

        qulonglong totalSamples = 0;
        double totalTimeSec = 0.0;
        qulonglong last1sSamples = 0;
        double lastDtSec = 0.0;

        {
            QMutexLocker lk(&pipeMu_);

            out = pipe_.process(t, *v);
            cap = pipe_.bias_capturing();
            has = pipe_.bias_has();

            if (cap != lastBiasCapturing_ || has != lastBiasHas_) {
                lastBiasCapturing_ = cap;
                lastBiasHas_ = has;
                emitBias = true;
            }

            st_total_samples_ += 1;

            uint64_t tn = (uint64_t)out.frame.t_ns;
            if (st_first_ns_ == 0) st_first_ns_ = tn;

            if (st_prev_ns_ != 0) st_last_dt_ns_ = tn - st_prev_ns_;
            else st_last_dt_ns_ = 0;

            st_prev_ns_ = tn;
            st_last_ns_ = tn;

            st_last1s_ts_.push_back(tn);
            while (!st_last1s_ts_.empty() && (tn - st_last1s_ts_.front()) > 1000000000ULL) {
                st_last1s_ts_.pop_front();
            }

            if (st_last_emit_ns_ == 0 || (tn - st_last_emit_ns_) >= 200000000ULL) {
                st_last_emit_ns_ = tn;
                emitStream = true;

                totalSamples = (qulonglong)st_total_samples_;
                totalTimeSec = (st_last_ns_ > st_first_ns_) ? (double)(st_last_ns_ - st_first_ns_) * 1e-9 : 0.0;
                last1sSamples = (qulonglong)st_last1s_ts_.size();
                lastDtSec = (double)st_last_dt_ns_ * 1e-9;
            }
        }

        if (emitBias) emit biasStateChanged(has, cap);
        if (emitStream) emit streamStats(totalSamples, totalTimeSec, last1sSamples, lastDtSec);

        ok_.fetch_add(1);

        if (csvOn_.load()) {
            QMutexLocker lk(&csvMu_);
            if (csv_ && (*csv_)) {
                if (!csvHeaderWritten_) {
                    (*csv_) << "t";
                    for (size_t i = 0; i < out.frame.x.size(); ++i) (*csv_) << ",ch" << i;
                    (*csv_) << "\n";
                    csvHeaderWritten_ = true;
                }
                uint64_t base = (csv_t0_ns_ ? csv_t0_ns_ : stream_t0);
                double ts = (static_cast<double>(out.frame.t_ns - base)) * 1e-9;
                (*csv_) << ts;
                for (size_t i = 0; i < out.frame.x.size(); ++i) (*csv_) << "," << out.frame.x[i];
                (*csv_) << "\n";
            }
        }

        QVector<float> qx;
        qx.reserve((int)out.frame.x.size());
        for (float f : out.frame.x) qx.push_back(f);

        emit frameReady((qulonglong)out.frame.t_ns, qx, false, 0.0f);

        static thread_local uint64_t lastStats = 0;
        if (t - lastStats > 500000000ULL) {
            lastStats = t;
            emit statsUpdated(ok_.load(), bad_.load());
        }
    }
}

void BleWorker::setPipelineConfig(hub::PipelineConfig cfg) {
    bool cap = false;
    bool has = false;
    {
        QMutexLocker lk(&pipeMu_);
        cfg_ = cfg;
        pipe_.set_config(cfg_);
        cap = pipe_.bias_capturing();
        has = pipe_.bias_has();
        lastBiasCapturing_ = cap;
        lastBiasHas_ = has;
    }
    emit biasStateChanged(has, cap);
}

void BleWorker::startBiasCapture(int frames) {
    if (n_ch_.load() == 0) return;

    bool cap = false;
    bool has = false;
    {
        QMutexLocker lk(&pipeMu_);
        pipe_.begin_bias_capture((size_t)std::max(frames, 1));
        cap = pipe_.bias_capturing();
        has = pipe_.bias_has();
        lastBiasCapturing_ = cap;
        lastBiasHas_ = has;
    }
    emit biasStateChanged(has, cap);
    emit statusText("Bias capture started");
}

void BleWorker::startCsv(QString path) {
    QMutexLocker lk(&csvMu_);
    csvPath_ = path;
    csv_.reset(new std::ofstream(csvPath_.toStdString(), std::ios::binary));
    csvHeaderWritten_ = false;
    csv_t0_ns_ = now_ns();
    csvOn_.store(true);
    emit statusText("CSV recording ON");
}

void BleWorker::stopCsv() {
    csvOn_.store(false);
    QMutexLocker lk(&csvMu_);
    if (csv_) {
        try { csv_->flush(); } catch (...) {}
        csv_.reset();
    }
    csvHeaderWritten_ = false;
    csv_t0_ns_ = 0;
    if (!csvPath_.isEmpty()) emit statusText("CSV recording OFF");
}

void BleWorker::saveBiasCsv(QString path) {
    std::vector<float> bias;
    {
        QMutexLocker lk(&pipeMu_);
        if (!pipe_.bias_has()) {
            emit statusText("No stored bias");
            return;
        }
        const auto& b = pipe_.bias_vec();
        bias.assign(b.begin(), b.end());
    }

    std::ofstream ofs(path.toStdString(), std::ios::binary);
    if (!ofs) {
        emit statusText("Bias CSV open failed");
        return;
    }

    if (bias.empty()) {
        emit statusText("Bias empty");
        return;
    }

    ofs << "ch0";
    for (size_t i = 1; i < bias.size(); ++i) ofs << ",ch" << i;
    ofs << "\n";

    ofs << bias[0];
    for (size_t i = 1; i < bias.size(); ++i) ofs << "," << bias[i];
    ofs << "\n";

    emit statusText("Bias CSV saved");
}
