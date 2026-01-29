#include "PositionTrackingEngine.h"

PositionTrackingEngine::PositionTrackingEngine(QObject* parent) : QObject(parent) {}

void PositionTrackingEngine::setAlgorithm(QString id) {
    algoId_ = id.toStdString();
    algo_ = hub::pt::create_algorithm(algoId_);
    params_.clear();
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

void PositionTrackingEngine::onSample(qulonglong, QVector<float> x, bool, float) {
    if (!algo_) return;

    std::vector<float> s;
    s.reserve((size_t)x.size());
    for (auto v : x) s.push_back(v);

    hub::pt::Output out;
    bool ok = algo_->push_sample(s, out);
    if (!ok) return;

    emit outputReady(out.x, out.y, out.z, out.q1, out.q2, out.err, out.quiet, out.valid);
}
