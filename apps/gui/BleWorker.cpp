#include "BleWorker.h"
#include <algorithm>
#include <chrono>
#include <cstring>

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
        emit statusText("No Bluetooth adapter");
        return;
    }

    emit statusText("Scanning...");
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
        try {
            adapter_->scan_for(1000);
            auto results = adapter_->scan_get_results();

            QVector<DeviceInfo> list;

            {
                QMutexLocker lk(&periphMu_);
                peripherals_.clear();

                for (auto& p : results) {
                    auto name = p.identifier();
                    if (!starts_with(name, prefix_.toStdString())) continue;

                    peripherals_.push_back(p);

                    DeviceInfo d;
                    d.name = QString::fromStdString(p.identifier());
                    d.address = QString::fromStdString(p.address());
                    d.rssi = p.rssi();
                    list.push_back(d);
                }
            }

            emit scanUpdated(list);
        } catch (...) {
            emit statusText("Scan error");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(120));
    }
}

void BleWorker::connectToIndex(int index) {
    if (!adapter_) return;

    bool wasScanning = scanning_.load();
    stopScanning();

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
        if (connected_.load() && active_ && active_->address() == p.address()) {
            disconnectDevice();
            if (wasScanning) startScanning();
            return;
        }

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

        notifyStart();
        connected_.store(true);

        emit connected(QString::fromStdString(p.identifier()), QString::fromStdString(p.address()));
        emit statusText("Connected");
    } catch (...) {
        emit statusText("Connect failed");
    }

    if (wasScanning) startScanning();
}

void BleWorker::disconnectDevice() {
    if (!connected_.load()) return;

    try { notifyStop(); } catch (...) {}
    try { if (active_) active_->disconnect(); } catch (...) {}

    active_.reset();
    svc_.reset();
    chr_.reset();
    connected_.store(false);

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
}

void BleWorker::notifyStart() {
    if (!active_ || !svc_ || !chr_) return;

    uint64_t stream_t0 = now_ns();

    active_->notify(*svc_, *chr_, [this, stream_t0](SimpleBLE::ByteArray payload) {
        if (!connected_.load()) return;

        std::string chunk(payload.begin(), payload.end());
        auto lines = framer_.push(chunk);

        for (auto& line : lines) {
            auto v = parser_.parse_line(line);
            if (!v) { bad_.fetch_add(1); continue; }

            size_t n = v->size();

            if (n_ch_.load() == 0) {
                n_ch_.store(n);
                QMutexLocker lk(&pipeMu_);
                pipe_.ensure_initialized(n);
                if (weightsPending_.load() && weights_.size() == n) {
                    pipe_.set_model_weights(weights_);
                    weightsPending_.store(false);
                }
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

                // ---- stream stat (샘플 1개 = 프레임 1개) ----
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

            ok_.fetch_add(1);

            if (csvOn_.load()) {
                QMutexLocker lk(&csvMu_);
                if (csv_ && (*csv_)) {
                    if (!csvHeaderWritten_) {
                        (*csv_) << "t";
                        for (size_t i = 0; i < out.frame.x.size(); ++i) (*csv_) << ",ch" << i;
                        (*csv_) << ",model\n";
                        csvHeaderWritten_ = true;
                    }
                    uint64_t base = (csv_t0_ns_ ? csv_t0_ns_ : stream_t0);
                    double ts = (static_cast<double>(out.frame.t_ns - base)) * 1e-9;
                    (*csv_) << ts;
                    for (size_t i = 0; i < out.frame.x.size(); ++i) (*csv_) << "," << out.frame.x[i];
                    (*csv_) << "," << (out.model_valid ? out.model_out : 0.0f) << "\n";
                }
            }

            if (emitStream) {
                emit streamStats(totalSamples, totalTimeSec, last1sSamples, lastDtSec);
            }

            QVector<float> qx;
            qx.reserve((int)out.frame.x.size());
            for (float f : out.frame.x) qx.push_back(f);

            emit frameReady((qulonglong)out.frame.t_ns, qx, out.model_valid, out.model_out);

            static thread_local uint64_t lastStats = 0;
            if (t - lastStats > 500000000ULL) {
                lastStats = t;
                emit statsUpdated(ok_.load(), bad_.load());
            }
        }
    });
}

void BleWorker::notifyStop() {
    if (!active_ || !svc_ || !chr_) return;
    active_->unsubscribe(*svc_, *chr_);
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

void BleWorker::loadWeights(QString path) {
    auto w = hub::load_weights_csv_1line(path.toStdString());
    if (!w) {
        emit statusText("Weights load failed");
        return;
    }
    weights_ = *w;
    weightsPending_.store(true);

    size_t n = n_ch_.load();
    if (n != 0 && weights_.size() == n) {
        QMutexLocker lk(&pipeMu_);
        pipe_.set_model_weights(weights_);
        weightsPending_.store(false);
        emit statusText("Weights applied");
    } else {
        emit statusText("Weights loaded (pending)");
    }
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

    ofs << "ch";
    if (!bias.empty()) {
        ofs.seekp(0, std::ios::beg);
        ofs << "ch0";
        for (size_t i = 1; i < bias.size(); ++i) ofs << ",ch" << i;
        ofs << "\n";
        ofs << bias[0];
        for (size_t i = 1; i < bias.size(); ++i) ofs << "," << bias[i];
        ofs << "\n";
    }

    emit statusText("Bias CSV saved");
}
