#include "PositionTrackingEngine.h"

PositionTrackingEngine::PositionTrackingEngine(QObject* parent) : QObject(parent) {}

void PositionTrackingEngine::setAlgorithm(QString id) {
    algoId_ = id.toStdString();
    algo_ = hub::pt::create_algorithm(algoId_);
    params_.clear();
    sampleBuf_.clear();
    lastStatusEmitNs_ = 0;
    lastAlgoCallNs_ = 0;
    minAlgoIntervalNs_ = 0;
    if (algoId_.find("PF_LA_znotfixed") != std::string::npos) {
        minAlgoIntervalNs_ = 10000000ULL;
    }
    if (!algo_) return;
    params_ = algo_->defaults();
    algo_->set_params(params_);
    algo_->reset();
}

void PositionTrackingEngine::setParams(QVector<double> params) {
    if (!algo_) return;
    params_.assign(params.begin(), params.end());
    algo_->set_params(params_);
}

void PositionTrackingEngine::reset() {
    if (algo_) algo_->reset();
}

void PositionTrackingEngine::onSample(qulonglong t_ns, QVector<float> x, bool, float) {
    if (!algo_) return;

    if (algo_->N() > 0 && x.size() != algo_->N()) {
        if (lastStatusEmitNs_ == 0 || (t_ns - lastStatusEmitNs_) > 500000000ULL) {
            lastStatusEmitNs_ = t_ns;
            emit statusReady(QString("Channel mismatch: expected %1, got %2").arg(algo_->N()).arg(x.size()));
        }
        return;
    }

    if (minAlgoIntervalNs_ > 0 && lastAlgoCallNs_ != 0 && (t_ns - lastAlgoCallNs_) < minAlgoIntervalNs_) {
        return;
    }
    lastAlgoCallNs_ = t_ns;

    sampleBuf_.resize((size_t)x.size());
    for (int i = 0; i < x.size(); ++i) sampleBuf_[(size_t)i] = x[i];

    hub::pt::Output out;
    bool ok = algo_->push_sample((uint64_t)t_ns, sampleBuf_, out);
    if (!ok) return;

    emit outputReady(out.x, out.y, out.z, out.confidence, out.q1, out.q2, out.err, out.quiet, out.valid);
}
