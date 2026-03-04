#include "hub/Pipeline.h"

namespace hub {

void Pipeline::reset() {
    n_ch_ = 0;

    ma_inited_.clear();
    ma_pos_.clear();
    ma_ring_.clear();
    ma_sum_.clear();

    ema_inited_.clear();
    ema_state_.clear();

    nx1_.clear();
    nx2_.clear();
    ny1_.clear();
    ny2_.clear();

    vrc_pos_.clear();
    vrc_cnt_.clear();
    vrc_ring_.clear();

    bias_.reset();
}

void Pipeline::set_config(const PipelineConfig& cfg) {
    cfg_ = cfg;

    // --- clamp / sanitize ---
    if (cfg_.ma_win < 1) cfg_.ma_win = 1;
    if (cfg_.ma_order < 1) cfg_.ma_order = 1;

    if (cfg_.ema_order < 1) cfg_.ema_order = 1;
    if (cfg_.ema_alpha < 0.0f) cfg_.ema_alpha = 0.0f;
    if (cfg_.ema_alpha > 1.0f) cfg_.ema_alpha = 1.0f;

    if (cfg_.notch_order < 1) cfg_.notch_order = 1;

    if (cfg_.vrc_n < 2) cfg_.vrc_n = 2;
    if (cfg_.vrc_order < 1) cfg_.vrc_order = 1;
    if (cfg_.vrc_rc <= 1e-12) cfg_.vrc_rc = 1e-12;

    update_notch_coeff();
}

void Pipeline::ensure_initialized(size_t n_ch) {
    if (n_ch == 0) return;

    if (n_ch_ != n_ch) {
        n_ch_ = n_ch;

        // Filters depend on channel count; wipe their state
        ma_inited_.clear();
        ma_pos_.clear();
        ma_ring_.clear();
        ma_sum_.clear();

        ema_inited_.clear();
        ema_state_.clear();

        nx1_.clear();
        nx2_.clear();
        ny1_.clear();
        ny2_.clear();

        vrc_pos_.clear();
        vrc_cnt_.clear();
        vrc_ring_.clear();
    }

    bias_.configure(n_ch_);
    ensure_ma();
    ensure_ema();
    ensure_notch();
    ensure_vrc();
}

void Pipeline::ensure_ma() {
    if (!cfg_.enable_ma) return;
    if (n_ch_ == 0) return;

    size_t win = cfg_.ma_win;
    if (win < 1) win = 1;

    size_t order = cfg_.ma_order;
    if (order < 1) order = 1;

    const size_t need_ring = order * win * n_ch_;
    const size_t need_sum = order * n_ch_;

    if (ma_ring_.size() != need_ring || ma_sum_.size() != need_sum ||
        ma_pos_.size() != order || ma_inited_.size() != order) {
        ma_ring_.assign(need_ring, 0.0f);
        ma_sum_.assign(need_sum, 0.0);
        ma_pos_.assign(order, 0);
        ma_inited_.assign(order, 0);
    }
}

void Pipeline::ensure_ema() {
    if (!cfg_.enable_ema) return;
    if (n_ch_ == 0) return;

    size_t order = cfg_.ema_order;
    if (order < 1) order = 1;

    const size_t need = order * n_ch_;

    if (ema_state_.size() != need || ema_inited_.size() != order) {
        ema_state_.assign(need, 0.0f);
        ema_inited_.assign(order, 0);
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

    size_t order = cfg_.notch_order;
    if (order < 1) order = 1;

    const size_t need = order * n_ch_;
    if (nx1_.size() != need) {
        nx1_.assign(need, 0.0);
        nx2_.assign(need, 0.0);
        ny1_.assign(need, 0.0);
        ny2_.assign(need, 0.0);
    }
}

void Pipeline::ensure_vrc() {
    if (!cfg_.enable_vrc) return;
    if (n_ch_ == 0) return;

    size_t order = cfg_.vrc_order;
    if (order < 1) order = 1;

    size_t n = cfg_.vrc_n;
    if (n < 2) n = 2;

    const size_t meta = order * n_ch_;
    const size_t need_ring = meta * n;

    if (vrc_pos_.size() != meta || vrc_cnt_.size() != meta || vrc_ring_.size() != need_ring) {
        vrc_pos_.assign(meta, 0);
        vrc_cnt_.assign(meta, 0);
        vrc_ring_.assign(need_ring, 0.0f);
    }
}

PipelineOut Pipeline::process(uint64_t t_ns, const std::vector<float>& in) {
    PipelineOut out;
    out.frame.t_ns = t_ns;
    out.frame.x = in;

    if (n_ch_ == 0) ensure_initialized(in.size());
    if (in.size() != n_ch_) ensure_initialized(in.size());

    auto& x = out.frame.x;

    // ---- Moving Average (cascaded) ----
    if (cfg_.enable_ma) {
        ensure_ma();

        size_t win = cfg_.ma_win;
        if (win < 1) win = 1;

        size_t order = cfg_.ma_order;
        if (order < 1) order = 1;

        for (size_t st = 0; st < order; ++st) {
            uint8_t inited = ma_inited_[st];
            size_t pos = ma_pos_[st];

            double* sum = ma_sum_.data() + (st * n_ch_);
            float* ring = ma_ring_.data() + (st * win * n_ch_);

            if (!inited) {
                // Initialize so the average output equals current input (no startup spike)
                for (size_t ch = 0; ch < n_ch_; ++ch) {
                    sum[ch] = (double)win * (double)x[ch];
                }
                for (size_t k = 0; k < win; ++k) {
                    for (size_t ch = 0; ch < n_ch_; ++ch) {
                        ring[k * n_ch_ + ch] = x[ch];
                    }
                }
                pos = 0;
                inited = 1;
            } else {
                size_t base = pos * n_ch_;
                for (size_t ch = 0; ch < n_ch_; ++ch) {
                    float oldv = ring[base + ch];
                    ring[base + ch] = x[ch];
                    sum[ch] += (double)x[ch] - (double)oldv;
                    x[ch] = (float)(sum[ch] / (double)win);
                }
                pos = (pos + 1) % win;
            }

            ma_pos_[st] = pos;
            ma_inited_[st] = inited;
        }
    }

    // ---- Exponential Moving Average (cascaded) ----
    if (cfg_.enable_ema) {
        ensure_ema();

        float a = cfg_.ema_alpha;
        if (a < 0.0f) a = 0.0f;
        if (a > 1.0f) a = 1.0f;

        size_t order = cfg_.ema_order;
        if (order < 1) order = 1;

        for (size_t st = 0; st < order; ++st) {
            uint8_t inited = ema_inited_[st];
            float* y = ema_state_.data() + (st * n_ch_);

            if (!inited) {
                for (size_t ch = 0; ch < n_ch_; ++ch) y[ch] = x[ch];
                inited = 1;
            } else {
                for (size_t ch = 0; ch < n_ch_; ++ch) {
                    y[ch] = a * x[ch] + (1.0f - a) * y[ch];
                }
            }

            for (size_t ch = 0; ch < n_ch_; ++ch) x[ch] = y[ch];
            ema_inited_[st] = inited;
        }
    }

    // ---- Notch (cascaded biquad) ----
    if (cfg_.enable_notch) {
        ensure_notch();
        update_notch_coeff();

        size_t order = cfg_.notch_order;
        if (order < 1) order = 1;

        for (size_t st = 0; st < order; ++st) {
            const size_t off = st * n_ch_;
            for (size_t ch = 0; ch < n_ch_; ++ch) {
                double xn = (double)x[ch];
                double yn = b0_ * xn + b1_ * nx1_[off + ch] + b2_ * nx2_[off + ch]
                            - a1_ * ny1_[off + ch] - a2_ * ny2_[off + ch];

                nx2_[off + ch] = nx1_[off + ch];
                nx1_[off + ch] = xn;
                ny2_[off + ch] = ny1_[off + ch];
                ny1_[off + ch] = yn;

                x[ch] = (float)yn;
            }
        }
    }

    // ---- V/RC + dV/dt (cascaded) ----
    if (cfg_.enable_vrc) {
        ensure_vrc();

        size_t n = cfg_.vrc_n;
        if (n < 2) n = 2;

        size_t order = cfg_.vrc_order;
        if (order < 1) order = 1;

        double RC = cfg_.vrc_rc;
        if (RC <= 1e-12) RC = 1e-12;

        // Use cfg_.fs_hz as the best available sampling rate (GUI auto-sets it from stream stats)
        double fs = cfg_.fs_hz;
        if (fs <= 1e-6) fs = 200.0;

        for (size_t st = 0; st < order; ++st) {
            const size_t meta_off = st * n_ch_;

            for (size_t ch = 0; ch < n_ch_; ++ch) {
                const size_t meta_i = meta_off + ch;

                size_t pos = vrc_pos_[meta_i];
                size_t cnt = (size_t)vrc_cnt_[meta_i];

                // ring base for this (stage, ch)
                float* ring = vrc_ring_.data() + (meta_i * n);

                // push current sample
                ring[pos] = x[ch];
                pos = (pos + 1) % n;
                if (cnt < n) cnt++;

                vrc_pos_[meta_i] = pos;
                vrc_cnt_[meta_i] = (uint16_t)cnt;

                // slope by linear regression on last cnt samples (oldest..newest)
                double dvdt = 0.0;
                if (cnt >= 2) {
                    const size_t m = cnt;
                    const double mean_i = (double)(m - 1) * 0.5;

                    double denom = 0.0;
                    double numer = 0.0;

                    const size_t start = (cnt == n) ? pos : 0; // pos now points to oldest when full
                    for (size_t i = 0; i < m; ++i) {
                        const size_t idx = (start + i) % n;
                        const double di = (double)i - mean_i;
                        denom += di * di;
                        numer += di * (double)ring[idx];
                    }

                    if (denom > 1e-12) {
                        const double slope_per_sample = numer / denom;
                        dvdt = slope_per_sample * fs;
                    }
                }

                const double y = ((double)x[ch]) / RC + dvdt;
                x[ch] = (float)y;
            }
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
