#include "hub/filters/Bias.h"
#include <algorithm>

namespace hub {

void BiasCorrector::reset() {
    n_ch_ = 0;
    capturing_ = false;
    has_bias_ = false;
    cap_target_ = 0;
    cap_count_ = 0;
    acc_.clear();
    bias_.clear();
}

void BiasCorrector::configure(size_t n_ch) {
    if (n_ch == 0) {
        reset();
        return;
    }

    if (n_ch_ == n_ch && n_ch_ != 0) {
        if (acc_.size() != n_ch_) acc_.resize(n_ch_, 0.0);
        if (bias_.size() != n_ch_) bias_.resize(n_ch_, 0.0f);
        return;
    }

    n_ch_ = n_ch;
    capturing_ = false;
    has_bias_ = false;
    cap_target_ = 0;
    cap_count_ = 0;
    acc_.assign(n_ch_, 0.0);
    bias_.assign(n_ch_, 0.0f);
}

void BiasCorrector::begin_capture(size_t frames) {
    if (n_ch_ == 0) return;
    if (frames < 1) frames = 1;

    capturing_ = true;
    cap_target_ = frames;
    cap_count_ = 0;

    if (acc_.size() != n_ch_) acc_.assign(n_ch_, 0.0);
    else std::fill(acc_.begin(), acc_.end(), 0.0);
}

void BiasCorrector::clear_bias() {
    has_bias_ = false;
    if (bias_.size() == n_ch_) {
        std::fill(bias_.begin(), bias_.end(), 0.0f);
    }
}

void BiasCorrector::set_bias(const std::vector<float>& bias) {
    if (bias.empty()) return;

    if (n_ch_ == 0 || bias.size() != n_ch_) {
        configure(bias.size());
    }

    bias_ = bias;
    has_bias_ = true;
    capturing_ = false;
    cap_target_ = 0;
    cap_count_ = 0;
}

void BiasCorrector::update_capture(const std::vector<float>& x) {
    if (!capturing_) return;
    if (x.size() != n_ch_) return;

    if (acc_.size() != n_ch_) acc_.assign(n_ch_, 0.0);

    for (size_t i = 0; i < n_ch_; ++i) acc_[i] += static_cast<double>(x[i]);
    cap_count_++;

    if (cap_count_ >= cap_target_) {
        double inv = 1.0 / static_cast<double>(cap_count_);
        if (bias_.size() != n_ch_) bias_.assign(n_ch_, 0.0f);
        for (size_t i = 0; i < n_ch_; ++i) bias_[i] = static_cast<float>(acc_[i] * inv);

        capturing_ = false;
        has_bias_ = true;
    }
}

void BiasCorrector::apply_inplace(std::vector<float>& x) const {
    if (!has_bias_) return;
    if (x.size() != n_ch_) return;

    for (size_t i = 0; i < n_ch_; ++i) x[i] -= bias_[i];
}

}
