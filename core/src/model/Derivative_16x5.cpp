#include "hub/model/Derivative_16x5.h"

#include <cmath>
#include <algorithm>

namespace hub::pt {

static double clamp01(double v) {
    if (v < 0.0) return 0.0;
    if (v > 1.0) return 1.0;
    return v;
}

static double safe_exp(double x) {
    if (x < -80.0) return 0.0;
    if (x > 80.0) return std::exp(80.0);
    return std::exp(x);
}

static double quantize(double v, double q) {
    if (!(q > 0.0)) return v;
    return std::round(v / q) * q;
}

Derivative_16x5::Derivative_16x5()
    : id_("Derivative_16x5"),
      buf_{},
      count_(0),
      head_(0),
      last_t_ns_(0),
      m_effective_(5),
      ema_alpha_(0.2),
      ema_degree_(3),
      range_gain_(1.0),
      noise_round_(1.0),
      sx_{},
      sy_{},
      has_bounds_(false),
      min_x_(0.0),
      max_x_(0.0),
      min_y_(0.0),
      max_y_(0.0),
      ema_inited_(false),
      x_ema_{},
      y_ema_{} {
    const double d = 19.1e-3;

    sx_[0]  = -1.5 * d; sy_[0]  = -1.5 * d;
    sx_[1]  =  0.5 * d; sy_[1]  = -1.5 * d;
    sx_[2]  =  1.5 * d; sy_[2]  = -1.5 * d;
    sx_[3]  =  0.5 * d; sy_[3]  = -0.5 * d;
    sx_[4]  =  1.5 * d; sy_[4]  = -0.5 * d;
    sx_[5]  =  0.5 * d; sy_[5]  =  0.5 * d;
    sx_[6]  =  1.5 * d; sy_[6]  =  0.5 * d;
    sx_[7]  =  0.5 * d; sy_[7]  =  1.5 * d;
    sx_[8]  =  1.5 * d; sy_[8]  =  1.5 * d;
    sx_[9]  = -0.5 * d; sy_[9]  =  1.5 * d;
    sx_[10] = -1.5 * d; sy_[10] =  1.5 * d;
    sx_[11] = -0.5 * d; sy_[11] =  0.5 * d;
    sx_[12] = -1.5 * d; sy_[12] =  0.5 * d;
    sx_[13] = -0.5 * d; sy_[13] = -0.5 * d;
    sx_[14] = -1.5 * d; sy_[14] = -0.5 * d;
    sx_[15] = -0.5 * d; sy_[15] = -1.5 * d;

    min_x_ = sx_[0];
    max_x_ = sx_[0];
    min_y_ = sy_[0];
    max_y_ = sy_[0];
    for (int i = 1; i < kN; ++i) {
        min_x_ = std::min(min_x_, sx_[i]);
        max_x_ = std::max(max_x_, sx_[i]);
        min_y_ = std::min(min_y_, sy_[i]);
        max_y_ = std::max(max_y_, sy_[i]);
    }
    has_bounds_ = true;
}

const std::string& Derivative_16x5::id() const {
    return id_;
}

int Derivative_16x5::N() const {
    return kN;
}

int Derivative_16x5::M() const {
    return kM;
}

std::vector<hub::pt::ParamDesc> Derivative_16x5::params() const {
    return {
        hub::pt::ParamDesc{ "m", "M (samples)", 2.0, 5.0, 5.0, 1.0, 0, false },
        hub::pt::ParamDesc{ "ema_alpha", "EMA scale", 0.01, 1.0, 0.20, 0.01, 2, false },
        hub::pt::ParamDesc{ "ema_degree", "EMA degree", 1.0, 5.0, 3.0, 1.0, 0, false },
        hub::pt::ParamDesc{ "range_gain", "Range gain", 0.50, 3.00, 1.00, 0.05, 2, false },
        hub::pt::ParamDesc{ "noise_round", "Noise rounding", 0.0, 5.0, 1.0, 0.1, 1, false }
    };
}

std::vector<double> Derivative_16x5::defaults() const {
    return { 5.0, 0.20, 3.0, 1.0, 1.0 };
}

void Derivative_16x5::set_params(const std::vector<double>& values) {
    if (values.size() >= 1) {
        long long mv = static_cast<long long>(std::llround(values[0]));
        if (mv < 2) mv = 2;
        if (mv > 5) mv = 5;
        m_effective_ = static_cast<int>(mv);
    }
    if (values.size() >= 2) {
        double a = values[1];
        if (a < 0.01) a = 0.01;
        if (a > 1.0) a = 1.0;
        ema_alpha_ = a;
    }
    if (values.size() >= 3) {
        long long dg = static_cast<long long>(std::llround(values[2]));
        if (dg < 1) dg = 1;
        if (dg > kEmaMaxDegree) dg = kEmaMaxDegree;
        ema_degree_ = static_cast<int>(dg);
    }
    if (values.size() >= 4) {
        double g = values[3];
        if (g < 0.50) g = 0.50;
        if (g > 3.00) g = 3.00;
        range_gain_ = g;
    }
    if (values.size() >= 5) {
        double q = values[4];
        if (q < 0.0) q = 0.0;
        if (q > 5.0) q = 5.0;
        noise_round_ = q;
    }
}

void Derivative_16x5::reset() {
    count_ = 0;
    head_ = 0;
    last_t_ns_ = 0;
    m_effective_ = 5;
    ema_alpha_ = 0.2;
    ema_degree_ = 3;
    range_gain_ = 1.0;
    noise_round_ = 1.0;
    ema_inited_ = false;
    for (auto& v : x_ema_) v = 0.0;
    for (auto& v : y_ema_) v = 0.0;
    for (auto& row : buf_) {
        for (float& v : row) v = 0.0f;
    }
}

const std::array<float, Derivative_16x5::kN>& Derivative_16x5::at_age(int age) const {
    int idx = head_ - age;
    while (idx < 0) idx += kM;
    idx %= kM;
    return buf_[idx];
}

void Derivative_16x5::fill_output_quiet(hub::pt::Output& out) const {
    out.x = 0.0;
    out.y = 0.0;
    out.confidence = 0.0;
    out.valid = false;
    out.quiet = true;
}

bool Derivative_16x5::push_sample(uint64_t t_ns, const std::vector<float>& sample, hub::pt::Output& out) {
    if (static_cast<int>(sample.size()) != kN) {
        return false;
    }

    const double tau_s = 0.05;
    const double fallback_dt_s = 1.0 / 105.0;

    double dt_s = fallback_dt_s;
    if (last_t_ns_ != 0 && t_ns > last_t_ns_) {
        dt_s = static_cast<double>(t_ns - last_t_ns_) * 1e-9;
        if (!(dt_s > 0.0)) dt_s = fallback_dt_s;
        if (dt_s > 0.2) dt_s = fallback_dt_s;
    }
    last_t_ns_ = t_ns;

    for (int ch = 0; ch < kN; ++ch) {
        buf_[head_][ch] = sample[ch];
    }
    head_ = (head_ + 1) % kM;
    if (count_ < kM) ++count_;

    if (count_ < kM) {
        fill_output_quiet(out);
        return false;
    }

    const int m_eff = std::max(2, std::min(kM, m_effective_));
    const int span = m_eff - 1;

    const double decay = safe_exp(-dt_s / tau_s);
    const double decay_span = std::pow(decay, static_cast<double>(span));

    const double noise_amp = 0.5;
    const double noise_delta = 0.6;
    const double amp_weight = 0.25;

    double sum_w = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;

    const auto& newest = at_age(1);
    const auto& older  = at_age(1 + span);

    const double q = noise_round_;

    for (int ch = 0; ch < kN; ++ch) {
        double x_new = quantize(static_cast<double>(newest[ch]), q);
        double x_old = quantize(static_cast<double>(older[ch]), q);

        double delta = x_new - x_old * decay_span;
        delta = quantize(delta, q);

        double w = std::abs(delta) - noise_delta;
        if (w < 0.0) w = 0.0;

        double a = std::abs(x_new) - noise_amp;
        if (a < 0.0) a = 0.0;

        w += amp_weight * a;

        if (w > 0.0) {
            sum_w += w;
            sum_x += sx_[ch] * w;
            sum_y += sy_[ch] * w;
        }
    }

    const double quiet_thr = 0.35;
    const double valid_thr = 0.80;
    const double conf_scale = 4.0;

    if (sum_w <= 0.0) {
        if (!ema_inited_) {
            out.x = 0.0;
            out.y = 0.0;
        } else {
            const int deg = std::max(1, std::min(kEmaMaxDegree, ema_degree_));
            out.x = x_ema_[deg - 1];
            out.y = y_ema_[deg - 1];
        }
        out.confidence = 0.0;
        out.valid = false;
        out.quiet = true;
        return true;
    }

    double x_est = sum_x / sum_w;
    double y_est = sum_y / sum_w;

    x_est *= range_gain_;
    y_est *= range_gain_;

    if (has_bounds_) {
        x_est = std::min(std::max(x_est, min_x_ * range_gain_), max_x_ * range_gain_);
        y_est = std::min(std::max(y_est, min_y_ * range_gain_), max_y_ * range_gain_);
    }

    const double conf = clamp01(1.0 - safe_exp(-sum_w / conf_scale));
    const bool quiet = (sum_w < quiet_thr);
    const bool valid = (sum_w >= valid_thr);

    const int deg = std::max(1, std::min(kEmaMaxDegree, ema_degree_));
    const double a = ema_alpha_;

    if (!ema_inited_) {
        for (int i = 0; i < kEmaMaxDegree; ++i) {
            x_ema_[i] = x_est;
            y_ema_[i] = y_est;
        }
        ema_inited_ = true;
    } else {
        x_ema_[0] += a * (x_est - x_ema_[0]);
        y_ema_[0] += a * (y_est - y_ema_[0]);
        for (int i = 1; i < deg; ++i) {
            x_ema_[i] += a * (x_ema_[i - 1] - x_ema_[i]);
            y_ema_[i] += a * (y_ema_[i - 1] - y_ema_[i]);
        }
    }

    out.x = x_ema_[deg - 1];
    out.y = y_ema_[deg - 1];
    out.confidence = clamp01(conf);
    out.valid = valid;
    out.quiet = quiet;

    return true;
}

HUB_PT_REGISTER_ALGORITHM(Derivative_16x5)

}
