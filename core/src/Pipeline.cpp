#include "hub/Pipeline.h"

namespace hub {

void Pipeline::reset() {
    n_ch_ = 0;

    ma_inited_ = false;
    ma_pos_ = 0;
    ma_ring_.clear();
    ma_sum_.clear();

    ema_inited_ = false;
    ema_state_.clear();

    notch_inited_ = false;
    nx1_.clear(); nx2_.clear(); ny1_.clear(); ny2_.clear();

    bias_.reset();
}

void Pipeline::set_config(const PipelineConfig& cfg) {
    cfg_ = cfg;
    if (cfg_.ma_win < 1) cfg_.ma_win = 1;
    update_notch_coeff();
}

void Pipeline::ensure_initialized(size_t n_ch) {
    if (n_ch == 0) return;

    if (n_ch_ != n_ch) {
        n_ch_ = n_ch;
        ma_inited_ = false;
        ema_inited_ = false;
        notch_inited_ = false;

        ma_pos_ = 0;
        ma_ring_.clear();
        ma_sum_.clear();
        ema_state_.clear();
        nx1_.clear(); nx2_.clear(); ny1_.clear(); ny2_.clear();
    }

    bias_.configure(n_ch_);
    ensure_ma();
    ensure_ema();
    ensure_notch();
}

void Pipeline::ensure_ma() {
    if (!cfg_.enable_ma) return;
    if (n_ch_ == 0) return;

    size_t win = cfg_.ma_win;
    if (win < 1) win = 1;

    size_t need = n_ch_ * win;
    if (ma_ring_.size() != need) {
        ma_ring_.assign(need, 0.0f);
        ma_sum_.assign(n_ch_, 0.0);
        ma_pos_ = 0;
        ma_inited_ = false;
    }
}

void Pipeline::ensure_ema() {
    if (!cfg_.enable_ema) return;
    if (n_ch_ == 0) return;
    if (ema_state_.size() != n_ch_) {
        ema_state_.assign(n_ch_, 0.0f);
        ema_inited_ = false;
    }
}

void Pipeline::update_notch_coeff() {
    if (!cfg_.enable_notch) return;

    double fs = cfg_.fs_hz;
    double f0 = cfg_.notch_f0;
    double Q  = cfg_.notch_q;

    if (fs <= 1e-6) fs = 200.0;
    if (f0 <= 1e-6) f0 = 60.0;
    if (Q  <= 1e-6) Q  = 30.0;

    double w0 = 2.0 * 3.14159265358979323846 * (f0 / fs);
    double c = std::cos(w0);
    double s = std::sin(w0);
    double alpha = s / (2.0 * Q);

    double b0 = 1.0;
    double b1 = -2.0 * c;
    double b2 = 1.0;
    double a0 = 1.0 + alpha;
    double a1 = -2.0 * c;
    double a2 = 1.0 - alpha;

    b0_ = b0 / a0;
    b1_ = b1 / a0;
    b2_ = b2 / a0;
    a1_ = a1 / a0;
    a2_ = a2 / a0;
}

void Pipeline::ensure_notch() {
    if (!cfg_.enable_notch) return;
    if (n_ch_ == 0) return;

    if (nx1_.size() != n_ch_) {
        nx1_.assign(n_ch_, 0.0);
        nx2_.assign(n_ch_, 0.0);
        ny1_.assign(n_ch_, 0.0);
        ny2_.assign(n_ch_, 0.0);
        notch_inited_ = true;
    }
}

PipelineOut Pipeline::process(uint64_t t_ns, const std::vector<float>& in) {
    PipelineOut out;
    out.frame.t_ns = t_ns;
    out.frame.x = in;

    if (n_ch_ == 0) ensure_initialized(in.size());
    if (in.size() != n_ch_) ensure_initialized(in.size());

    auto& x = out.frame.x;

    // MA
    if (cfg_.enable_ma) {
        ensure_ma();
        size_t win = cfg_.ma_win;
        if (!ma_inited_) {
            for (size_t ch = 0; ch < n_ch_; ++ch) {
                ma_sum_[ch] = (double)win * (double)x[ch];
            }
            for (size_t k = 0; k < win; ++k) {
                for (size_t ch = 0; ch < n_ch_; ++ch) {
                    ma_ring_[k * n_ch_ + ch] = x[ch];
                }
            }
            ma_pos_ = 0;
            ma_inited_ = true;
        } else {
            size_t base = ma_pos_ * n_ch_;
            for (size_t ch = 0; ch < n_ch_; ++ch) {
                float oldv = ma_ring_[base + ch];
                ma_ring_[base + ch] = x[ch];
                ma_sum_[ch] += (double)x[ch] - (double)oldv;
                x[ch] = (float)(ma_sum_[ch] / (double)win);
            }
            ma_pos_ = (ma_pos_ + 1) % win;
        }
    }

    // EMA
    if (cfg_.enable_ema) {
        ensure_ema();
        float a = cfg_.ema_alpha;
        if (!ema_inited_) {
            ema_state_ = x;
            ema_inited_ = true;
        } else {
            for (size_t ch = 0; ch < n_ch_; ++ch) {
                ema_state_[ch] = a * x[ch] + (1.0f - a) * ema_state_[ch];
            }
            x = ema_state_;
        }
    }

    // Notch
    if (cfg_.enable_notch) {
        ensure_notch();
        update_notch_coeff();
        for (size_t ch = 0; ch < n_ch_; ++ch) {
            double xn = (double)x[ch];
            double yn = b0_ * xn + b1_ * nx1_[ch] + b2_ * nx2_[ch]
                        - a1_ * ny1_[ch] - a2_ * ny2_[ch];

            nx2_[ch] = nx1_[ch];
            nx1_[ch] = xn;
            ny2_[ch] = ny1_[ch];
            ny1_[ch] = yn;

            x[ch] = (float)yn;
        }
    }

    // Bias capture/update
    if (bias_.capturing()) bias_.update_capture(x);

    // Bias apply
    if (cfg_.enable_bias) bias_.apply_inplace(x);

    return out;
}

void Pipeline::begin_bias_capture(size_t frames) {
    bias_.begin_capture(frames);
}

}
