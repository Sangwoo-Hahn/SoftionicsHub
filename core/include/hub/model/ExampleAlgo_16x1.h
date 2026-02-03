#ifndef HUB_MODEL_EXAMPLEALGO_16X1_H
#define HUB_MODEL_EXAMPLEALGO_16X1_H

#include "hub/model/PositionTrackingRegistry.h"

#include <algorithm>
#include <cmath>

namespace hub::pt {

class ExampleAlgo_16x1 final : public IAlgorithm {
public:
    ExampleAlgo_16x1() {
        set_params(defaults());
        reset();
    }

    const std::string& id() const override {
        static std::string s = "ExampleAlgo_16x1";
        return s;
    }

    int N() const override { return 16; }
    int M() const override { return 1; }

    std::vector<ParamDesc> params() const override {
        return {
            {"scale", "Scale", 0.0, 0.2, 0.03, 0.001, 6, false},
            {"gain", "Conf gain", 0.0, 50.0, 5.0, 0.1, 4, false},
            {"min_conf", "Min conf", 0.0, 1.0, 0.15, 0.01, 4, false}
        };
    }

    std::vector<double> defaults() const override {
        auto p = params();
        std::vector<double> d;
        d.reserve(p.size());
        for (auto& x : p) d.push_back(x.defv);
        return d;
    }

    void set_params(const std::vector<double>& values) override {
        auto d = defaults();
        params_ = values;
        if (params_.size() < d.size()) params_.resize(d.size(), 0.0);

        scale_ = params_[0];
        gain_ = params_[1];
        min_conf_ = params_[2];

        if (scale_ < 0.0) scale_ = 0.0;
        if (gain_ < 0.0) gain_ = 0.0;
        min_conf_ = std::clamp(min_conf_, 0.0, 1.0);

        params_[0] = scale_;
        params_[1] = gain_;
        params_[2] = min_conf_;
    }

    void reset() override {}

    bool push_sample(uint64_t, const std::vector<float>& sample, Output& out) override {
        if (sample.size() != 16) return false;

        double s0 = 0.0;
        double s1 = 0.0;
        double se = 0.0;
        double so = 0.0;
        double e = 0.0;

        for (int i = 0; i < 16; ++i) {
            double v = (double)sample[(size_t)i];
            double av = std::abs(v);
            e += av;
            if (i < 8) s0 += v;
            else s1 += v;
            if ((i & 1) == 0) se += v;
            else so += v;
        }

        double denomx = std::abs(s0) + std::abs(s1) + 1e-9;
        double denomy = std::abs(se) + std::abs(so) + 1e-9;

        double dx = (s0 - s1) / denomx;
        double dy = (se - so) / denomy;

        double mean_abs = e / 16.0;
        double conf = 1.0 - std::exp(-gain_ * mean_abs);
        conf = std::clamp(conf, 0.0, 1.0);

        out.x = scale_ * dx;
        out.y = scale_ * dy;
        out.z = 0.0;
        out.confidence = conf;
        out.q1 = dx;
        out.q2 = dy;
        out.err = 1.0 - conf;
        out.quiet = conf < min_conf_;
        out.valid = true;

        return true;
    }

private:
    std::vector<double> params_;

    double scale_ = 0.03;
    double gain_ = 5.0;
    double min_conf_ = 0.15;
};

} // namespace hub::pt

#endif
