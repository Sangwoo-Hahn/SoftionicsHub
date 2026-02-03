#include "hub/model/Derivative2_16x5.h"

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

static double denom_for_len(int L) {
    if (L <= 1) return 1.0;
    if (L == 2) return 0.5;
    if (L == 3) return 2.0;
    if (L == 4) return 5.0;
    return 10.0;
}

Derivative2_16x5::Derivative2_16x5()
    : id_("Derivative2_16x5"),
      buf_{},
      count_(0),
      head_(0),
      last_t_ns_(0),
      m_effective_(5),
      ema_alpha_(0.2),
      ema_degree_(1),
      range_gain_(1.0),
      noise_round_(1.0),
      motion_deadband_(1.0),
      prior_strength_(6.0),
      hold_w_(0.8),
      conf_scale_(6.0),
      sx_{},
      sy_{},
      has_bounds_(false),
      min_x_(0.0),
      max_x_(0.0),
      min_y_(0.0),
      max_y_(0.0),
      has_last_pos_(false),
      last_out_x_(0.0),
      last_out_y_(0.0),
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

const std::string& Derivative2_16x5::id() const {
    return id_;
}

int Derivative2_16x5::N() const {
    return kN;
}

int Derivative2_16x5::M() const {
    return kM;
}

std::vector<hub::pt::ParamDesc> Derivative2_16x5::params() const {
    return {
        hub::pt::ParamDesc{ "m", "M (samples)", 2.0, 5.0, 5.0, 1.0, 0, false },
        hub::pt::ParamDesc{ "ema_alpha", "EMA scale", 0.0, 1.0, 0.20, 0.01, 2, false },
        hub::pt::ParamDesc{ "ema_degree", "EMA degree", 0.0, 8.0, 1.0, 1.0, 0, false },
        hub::pt::ParamDesc{ "range_gain", "Range gain", 0.1, 10.0, 1.00, 0.05, 2, false },
        hub::pt::ParamDesc{ "noise_round", "Noise rounding", 0.0, 20.0, 1.0, 0.1, 1, false },
        hub::pt::ParamDesc{ "motion_deadband", "Motion deadband", 0.0, 20.0, 1.0, 0.1, 1, false },
        hub::pt::ParamDesc{ "prior_strength", "Prior strength", 0.0, 50.0, 6.0, 0.5, 1, false },
        hub::pt::ParamDesc{ "hold_w", "Hold threshold", 0.0, 10.0, 0.80, 0.05, 2, false },
        hub::pt::ParamDesc{ "conf_scale", "Confidence scale", 0.1, 50.0, 6.0, 0.1, 1, false }
    };
}

std::vector<double> Derivative2_16x5::defaults() const {
    return { 5.0, 0.20, 1.0, 1.00, 1.0, 1.0, 6.0, 0.80, 6.0 };
}

void Derivative2_16x5::set_params(const std::vector<double>& values) {
    if (values.size() >= 1) {
        long long mv = static_cast<long long>(std::llround(values[0]));
        if (mv < 2) mv = 2;
        if (mv > 5) mv = 5;
        m_effective_ = static_cast<int>(mv);
    }
    if (values.size() >= 2) {
        double a = values[1];
        if (a < 0.0) a = 0.0;
        if (a > 1.0) a = 1.0;
        ema_alpha_ = a;
    }
    if (values.size() >= 3) {
        long long dg = static_cast<long long>(std::llround(values[2]));
        if (dg < 0) dg = 0;
        if (dg > kEmaMaxDegree) dg = kEmaMaxDegree;
        ema_degree_ = static_cast<int>(dg);
    }
    if (values.size() >= 4) {
        double g = values[3];
        if (g < 0.1) g = 0.1;
        if (g > 10.0) g = 10.0;
        range_gain_ = g;
    }
    if (values.size() >= 5) {
        double q = values[4];
        if (q < 0.0) q = 0.0;
        if (q > 20.0) q = 20.0;
        noise_round_ = q;
    }
    if (values.size() >= 6) {
        double d = values[5];
        if (d < 0.0) d = 0.0;
        if (d > 20.0) d = 20.0;
        motion_deadband_ = d;
    }
    if (values.size() >= 7) {
        double p = values[6];
        if (p < 0.0) p = 0.0;
        if (p > 50.0) p = 50.0;
        prior_strength_ = p;
    }
    if (values.size() >= 8) {
        double h = values[7];
        if (h < 0.0) h = 0.0;
        if (h > 10.0) h = 10.0;
        hold_w_ = h;
    }
    if (values.size() >= 9) {
        double c = values[8];
        if (c < 0.1) c = 0.1;
        if (c > 50.0) c = 50.0;
        conf_scale_ = c;
    }
}

void Derivative2_16x5::reset() {
    count_ = 0;
    head_ = 0;
    last_t_ns_ = 0;

    m_effective_ = 5;
    ema_alpha_ = 0.2;
    ema_degree_ = 1;
    range_gain_ = 1.0;
    noise_round_ = 1.0;

    motion_deadband_ = 1.0;
    prior_strength_ = 6.0;
    hold_w_ = 0.8;
    conf_scale_ = 6.0;

    has_last_pos_ = false;
    last_out_x_ = 0.0;
    last_out_y_ = 0.0;

    ema_inited_ = false;
    for (auto& v : x_ema_) v = 0.0;
    for (auto& v : y_ema_) v = 0.0;

    for (auto& row : buf_) {
        for (float& v : row) v = 0.0f;
    }
}

const std::array<float, Derivative2_16x5::kN>& Derivative2_16x5::at_age(int age) const {
    int idx = head_ - age;
    while (idx < 0) idx += kM;
    idx %= kM;
    return buf_[idx];
}

void Derivative2_16x5::fill_output_quiet(hub::pt::Output& out) const {
    out.x = has_last_pos_ ? last_out_x_ : 0.0;
    out.y = has_last_pos_ ? last_out_y_ : 0.0;
    out.confidence = 0.0;
    out.valid = false;
    out.quiet = true;
}

bool Derivative2_16x5::push_sample(uint64_t t_ns, const std::vector<float>& sample, hub::pt::Output& out) {
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
    const double mean_k = 0.5 * static_cast<double>(m_eff - 1);
    const double denom = denom_for_len(m_eff);

    const double decay = safe_exp(-dt_s / tau_s);

    double sum_w = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;

    const double q = noise_round_;
    const double dead = motion_deadband_;

    for (int ch = 0; ch < kN; ++ch) {
        double num = 0.0;
        double p = 1.0;

        for (int k = m_eff - 1; k >= 0; --k) {
            const int age = m_eff - k;
            double xk = quantize(static_cast<double>(at_age(age)[ch]), q);
            double xadj = xk * p;
            num += (static_cast<double>(k) - mean_k) * xadj;
            p *= decay;
        }

        double slope_idx = num / denom;
        slope_idx = quantize(slope_idx, q);

        double w = std::abs(slope_idx) - dead;
        if (w < 0.0) w = 0.0;

        if (w > 0.0) {
            sum_w += w;
            sum_x += sx_[ch] * w;
            sum_y += sy_[ch] * w;
        }
    }

    if (!(sum_w > 0.0)) {
        fill_output_quiet(out);
        return true;
    }

    const double conf = clamp01(1.0 - safe_exp(-sum_w / conf_scale_));

    if (has_last_pos_ && sum_w < hold_w_) {
        out.x = last_out_x_;
        out.y = last_out_y_;
        out.confidence = clamp01(conf);
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

    if (has_last_pos_) {
        const double pw = prior_strength_ * (1.0 - conf);
        const double denom2 = sum_w + pw;
        if (denom2 > 0.0) {
            x_est = (sum_w * x_est + pw * last_out_x_) / denom2;
            y_est = (sum_w * y_est + pw * last_out_y_) / denom2;
        }
    }

    double x_out = x_est;
    double y_out = y_est;

    const int deg = std::max(0, std::min(kEmaMaxDegree, ema_degree_));
    const double a = ema_alpha_;

    if (deg > 0 && a > 0.0 && a < 1.0) {
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
        x_out = x_ema_[deg - 1];
        y_out = y_ema_[deg - 1];
    } else if (deg > 0 && a >= 1.0) {
        if (!ema_inited_) {
            for (int i = 0; i < kEmaMaxDegree; ++i) {
                x_ema_[i] = x_est;
                y_ema_[i] = y_est;
            }
            ema_inited_ = true;
        } else {
            x_ema_[0] = x_est;
            y_ema_[0] = y_est;
            for (int i = 1; i < deg; ++i) {
                x_ema_[i] = x_ema_[i - 1];
                y_ema_[i] = y_ema_[i - 1];
            }
        }
        x_out = x_ema_[deg - 1];
        y_out = y_ema_[deg - 1];
    }

    last_out_x_ = x_out;
    last_out_y_ = y_out;
    has_last_pos_ = true;

    out.x = x_out;
    out.y = y_out;
    out.confidence = clamp01(conf);
    out.valid = (conf >= 0.35);
    out.quiet = (conf < 0.15);

    return true;
}

HUB_PT_REGISTER_ALGORITHM(Derivative2_16x5)

}
