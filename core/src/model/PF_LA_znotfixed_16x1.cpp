#include "hub/model/PF_LA_znotfixed_16x1.h"

namespace hub::pt {

PF_LA_znotfixed_16x1::PF_LA_znotfixed_16x1()
    : used_sids_{ {0, 1, 2, 4, 6, 7, 8, 9, 10, 12, 14, 15} }
    , xs_u_{ {-28.65, 9.55, 28.65, 28.65, 28.65, 9.55, 28.65, -9.55, -28.65, -28.65, -28.65, -9.55} }
    , ys_u_{ {-28.65, -28.65, -28.65, -9.55, 9.55, 28.65, 28.65, 28.65, 28.65, 9.55, -9.55, -28.65} }
    , k_dv_(0.0325)
    , deriv_window_ms_(25.0)
    , kprime_(-2.663)
    , alpha_(1.25)
    , z0_(2.0)
    , P_(250)
    , ess_ratio_(0.30)
    , respawn_ratio_move_(0.05)
    , pos_noise_xy_move_(1.8)
    , pos_noise_z_move_(2.5)
    , vel_noise_xy_move_(80.0)
    , vel_noise_z_move_(120.0)
    , pos_noise_xy_stop_(0.05)
    , pos_noise_z_stop_(0.05)
    , vel_noise_xy_stop_(1.0)
    , vel_noise_z_stop_(1.0)
    , stop_damp_(0.90)
    , jitter_xy_(1.0)
    , jitter_z_(1.5)
    , z_min_(2.0)
    , z_max_(200.0)
    , ridge_(1e-6)
    , lam_vx_(8e-3)
    , lam_vy_(8e-3)
    , lam_vz_(8e-3)
    , beta_kin_(2.0)
    , sigma_kin_(6.0)
    , inv_sigma_kin2_(1.0 / (sigma_kin_ * sigma_kin_))
    , sigma_scale_(1.05)
    , gate_ratio_(0.35)
    , vpen_(3e-6)
    , vpen_yz_(2e-5)
    , cond_thr_(1e6)
    , cond_gain_(2.5)
    , vel_mix_(0.02)
    , sigma_(1.0)
    , s_thr_(0.0)
    , median_s_norm_(0.0)
    , has_prev_sample_(false)
    , prev_sample_{}
    , dVdt_smooth_{}
    , prev_t_ns_(0)
    , prev_s_norm_(0.0)
    , prev_s_thr_(0.0)
    , sample_count_(0)
    , s_norm_buf_{}
    , s_norm_scratch_{}
    , s_norm_buf_fill_(0)
    , s_norm_buf_head_(0)
    , rng_(std::random_device{}())
    , uni_(0.0, 1.0)
    , has_spare_normal_(false)
    , spare_normal_(0.0) {
    ensure_storage();
    reset();
}

const std::string& PF_LA_znotfixed_16x1::id() const {
    static const std::string kId = "PF_LA_znotfixed_16x1";
    return kId;
}

int PF_LA_znotfixed_16x1::N() const {
    return kN;
}

int PF_LA_znotfixed_16x1::M() const {
    return kM;
}

std::vector<hub::pt::ParamDesc> PF_LA_znotfixed_16x1::params() const {
    return {
        hub::pt::ParamDesc{"k_dv", "k_dv", 0.0, 0.2, 0.0325, 0.0001, 4, false},
        hub::pt::ParamDesc{"deriv_ms", "deriv_ms", 1.0, 200.0, 25.0, 1.0, 0, false},
        hub::pt::ParamDesc{"kprime", "kprime", -50.0, 50.0, -2.663, 0.001, 4, false},
        hub::pt::ParamDesc{"alpha", "alpha", 0.1, 5.0, 1.25, 0.001, 4, false},
        hub::pt::ParamDesc{"z0", "z0", 0.0, 50.0, 2.0, 0.01, 3, false},
        hub::pt::ParamDesc{"P", "particles", 100.0, 2000.0, 250.0, 10.0, 0, false},
        hub::pt::ParamDesc{"ess_ratio", "ess_ratio", 0.05, 0.95, 0.30, 0.01, 2, false},
        hub::pt::ParamDesc{"respawn_ratio_move", "respawn_ratio_move", 0.0, 0.30, 0.05, 0.01, 2, false},
        hub::pt::ParamDesc{"pos_noise_xy_move", "pos_noise_xy_move", 0.0, 20.0, 1.8, 0.01, 2, false},
        hub::pt::ParamDesc{"pos_noise_z_move", "pos_noise_z_move", 0.0, 20.0, 2.5, 0.01, 2, false},
        hub::pt::ParamDesc{"vel_noise_xy_move", "vel_noise_xy_move", 0.0, 400.0, 80.0, 1.0, 1, false},
        hub::pt::ParamDesc{"vel_noise_z_move", "vel_noise_z_move", 0.0, 600.0, 120.0, 1.0, 1, false},
        hub::pt::ParamDesc{"pos_noise_xy_stop", "pos_noise_xy_stop", 0.0, 5.0, 0.05, 0.001, 3, false},
        hub::pt::ParamDesc{"pos_noise_z_stop", "pos_noise_z_stop", 0.0, 5.0, 0.05, 0.001, 3, false},
        hub::pt::ParamDesc{"vel_noise_xy_stop", "vel_noise_xy_stop", 0.0, 50.0, 1.0, 0.01, 2, false},
        hub::pt::ParamDesc{"vel_noise_z_stop", "vel_noise_z_stop", 0.0, 50.0, 1.0, 0.01, 2, false},
        hub::pt::ParamDesc{"stop_damp", "stop_damp", 0.0, 1.0, 0.90, 0.01, 2, false},
        hub::pt::ParamDesc{"jitter_xy", "jitter_xy", 0.0, 20.0, 1.0, 0.01, 2, false},
        hub::pt::ParamDesc{"jitter_z", "jitter_z", 0.0, 20.0, 1.5, 0.01, 2, false},
        hub::pt::ParamDesc{"z_min", "z_min", 0.0, 100.0, 2.0, 0.1, 2, false},
        hub::pt::ParamDesc{"z_max", "z_max", 10.0, 400.0, 200.0, 0.1, 2, false},
        hub::pt::ParamDesc{"ridge", "ridge", 1e-12, 1e-2, 1e-6, 1e-6, 6, true},
        hub::pt::ParamDesc{"lam_vx", "lam_vx", 0.0, 0.2, 8e-3, 1e-4, 6, true},
        hub::pt::ParamDesc{"lam_vy", "lam_vy", 0.0, 0.2, 8e-3, 1e-4, 6, true},
        hub::pt::ParamDesc{"lam_vz", "lam_vz", 0.0, 0.2, 8e-3, 1e-4, 6, true},
        hub::pt::ParamDesc{"beta_kin", "beta_kin", 0.0, 10.0, 2.0, 0.01, 2, false},
        hub::pt::ParamDesc{"sigma_kin", "sigma_kin", 0.1, 50.0, 6.0, 0.1, 2, false},
        hub::pt::ParamDesc{"sigma_scale", "sigma_scale", 0.1, 5.0, 1.05, 0.01, 2, false},
        hub::pt::ParamDesc{"gate_ratio", "gate_ratio", 0.0, 1.0, 0.35, 0.01, 2, false},
        hub::pt::ParamDesc{"vpen", "vpen", 0.0, 1e-3, 3e-6, 1e-6, 7, true},
        hub::pt::ParamDesc{"vpen_yz", "vpen_yz", 0.0, 1e-2, 2e-5, 1e-6, 7, true},
        hub::pt::ParamDesc{"cond_thr", "cond_thr", 1e3, 1e9, 1e6, 1e3, 0, true},
        hub::pt::ParamDesc{"cond_gain", "cond_gain", 0.0, 10.0, 2.5, 0.01, 2, false},
        hub::pt::ParamDesc{"vel_mix", "vel_mix", 0.0, 1.0, 0.02, 0.001, 3, false}
    };
}

std::vector<double> PF_LA_znotfixed_16x1::defaults() const {
    return {
        0.0325,
        25.0,
        -2.663,
        1.25,
        2.0,
        250.0,
        0.30,
        0.05,
        1.8,
        2.5,
        80.0,
        120.0,
        0.05,
        0.05,
        1.0,
        1.0,
        0.90,
        1.0,
        1.5,
        2.0,
        200.0,
        1e-6,
        8e-3,
        8e-3,
        8e-3,
        2.0,
        6.0,
        1.05,
        0.35,
        3e-6,
        2e-5,
        1e6,
        2.5,
        0.02
    };
}

void PF_LA_znotfixed_16x1::set_params(const std::vector<double>& values) {
    auto clampv = [&](double v, double lo, double hi) { return clamp(v, lo, hi); };

    const auto defs = defaults();
    std::vector<double> v(defs.size());
    for (size_t i = 0; i < defs.size(); ++i) {
        v[i] = (i < values.size()) ? values[i] : defs[i];
    }

    k_dv_ = clampv(v[0], 0.0, 0.2);
    deriv_window_ms_ = clampv(v[1], 1.0, 200.0);
    kprime_ = clampv(v[2], -50.0, 50.0);
    alpha_ = clampv(v[3], 0.1, 5.0);
    z0_ = clampv(v[4], 0.0, 50.0);

    int newP = static_cast<int>(std::llround(clampv(v[5], 100.0, 2000.0)));
    if (newP < 100) newP = 100;

    ess_ratio_ = clampv(v[6], 0.05, 0.95);
    respawn_ratio_move_ = clampv(v[7], 0.0, 0.30);

    pos_noise_xy_move_ = clampv(v[8], 0.0, 20.0);
    pos_noise_z_move_ = clampv(v[9], 0.0, 20.0);
    vel_noise_xy_move_ = clampv(v[10], 0.0, 400.0);
    vel_noise_z_move_ = clampv(v[11], 0.0, 600.0);

    pos_noise_xy_stop_ = clampv(v[12], 0.0, 5.0);
    pos_noise_z_stop_ = clampv(v[13], 0.0, 5.0);
    vel_noise_xy_stop_ = clampv(v[14], 0.0, 50.0);
    vel_noise_z_stop_ = clampv(v[15], 0.0, 50.0);

    stop_damp_ = clampv(v[16], 0.0, 1.0);
    jitter_xy_ = clampv(v[17], 0.0, 20.0);
    jitter_z_ = clampv(v[18], 0.0, 20.0);

    z_min_ = clampv(v[19], 0.0, 100.0);
    z_max_ = clampv(v[20], 10.0, 400.0);
    if (z_max_ < z_min_ + 1e-6) z_max_ = z_min_ + 1e-6;

    ridge_ = clampv(v[21], 1e-12, 1e-2);

    lam_vx_ = clampv(v[22], 0.0, 0.2);
    lam_vy_ = clampv(v[23], 0.0, 0.2);
    lam_vz_ = clampv(v[24], 0.0, 0.2);

    beta_kin_ = clampv(v[25], 0.0, 10.0);
    sigma_kin_ = clampv(v[26], 0.1, 50.0);
    inv_sigma_kin2_ = 1.0 / (sigma_kin_ * sigma_kin_ + kEps);

    sigma_scale_ = clampv(v[27], 0.1, 5.0);
    gate_ratio_ = clampv(v[28], 0.0, 1.0);

    vpen_ = clampv(v[29], 0.0, 1e-3);
    vpen_yz_ = clampv(v[30], 0.0, 1e-2);

    cond_thr_ = clampv(v[31], 1e3, 1e9);
    cond_gain_ = clampv(v[32], 0.0, 10.0);

    vel_mix_ = clampv(v[33], 0.0, 1.0);

    if (newP != P_) {
        P_ = newP;
        ensure_storage();
        init_particles();
    }

    update_median_and_scales();
}

void PF_LA_znotfixed_16x1::reset() {
    has_prev_sample_ = false;
    prev_t_ns_ = 0;
    prev_s_norm_ = 0.0;
    prev_s_thr_ = 0.0;
    sample_count_ = 0;

    dVdt_smooth_.fill(0.0);
    prev_sample_.fill(0.0f);

    s_norm_buf_fill_ = 0;
    s_norm_buf_head_ = 0;
    median_s_norm_ = 0.0;
    sigma_ = 1.0;
    s_thr_ = 0.0;

    init_particles();
}

bool PF_LA_znotfixed_16x1::push_sample(uint64_t t_ns, const std::vector<float>& sample, hub::pt::Output& out) {
    if (sample.size() != static_cast<size_t>(kN)) {
        set_output_fields(out, 0.0, 0.0, 0.0, false, true);
        return false;
    }

    ++sample_count_;

    double dt = 0.0;
    if (has_prev_sample_) {
        uint64_t dtns = (t_ns >= prev_t_ns_) ? (t_ns - prev_t_ns_) : 0;
        dt = static_cast<double>(dtns) * 1e-9;
        if (!std::isfinite(dt) || dt < 0.0) dt = 0.0;
        if (dt > 0.1) dt = 0.1;
    }

    double tau = deriv_window_ms_ * 1e-3;
    double a = 0.0;
    if (tau <= 0.0) {
        a = 1.0;
    } else if (dt > 0.0) {
        a = dt / (tau + dt);
    } else {
        a = 0.0;
    }

    for (int ch = 0; ch < kN; ++ch) {
        double vcur = static_cast<double>(sample[ch]);
        double dv = 0.0;
        if (has_prev_sample_ && dt > 1e-9) {
            dv = (vcur - static_cast<double>(prev_sample_[ch])) / dt;
            if (!std::isfinite(dv)) dv = 0.0;
        }
        dVdt_smooth_[ch] = (1.0 - a) * dVdt_smooth_[ch] + a * dv;
        if (!std::isfinite(dVdt_smooth_[ch])) dVdt_smooth_[ch] = 0.0;
    }

    std::array<double, kUsed> s_vec{};
    double s2 = 0.0;
    for (int i = 0; i < kUsed; ++i) {
        int ch = used_sids_[i];
        double sval = static_cast<double>(sample[ch]) + k_dv_ * dVdt_smooth_[ch];
        s_vec[i] = sval;
        s2 += sval * sval;
    }

    double s_norm = safe_sqrt(s2);
    push_snorm(s_norm);
    if ((sample_count_ % kMedianUpdateInterval) == 0 || sample_count_ == 1) {
        update_median_and_scales();
    }
    if (median_s_norm_ <= 0.0) {
        median_s_norm_ = s_norm;
        update_median_and_scales();
    }

    bool prev_stop_mode = false;
    if (has_prev_sample_) {
        prev_stop_mode = (prev_s_norm_ < prev_s_thr_);
    } else {
        prev_stop_mode = (s_norm < s_thr_);
    }

    const size_t Pn = particles_pos_.size();
    if (Pn == 0) {
        set_output_fields(out, 0.0, 0.0, 0.0, false, true);
        prev_t_ns_ = t_ns;
        for (int ch = 0; ch < kN; ++ch) prev_sample_[ch] = sample[ch];
        has_prev_sample_ = true;
        prev_s_norm_ = s_norm;
        prev_s_thr_ = s_thr_;
        return true;
    }

    for (size_t i = 0; i < Pn; ++i) {
        Vec3& pos = particles_pos_[i];
        Vec3& vel = particles_vel_[i];

        if (prev_stop_mode) {
            vel.x *= stop_damp_;
            vel.y *= stop_damp_;
            vel.z *= stop_damp_;
        }

        Vec3 dp{0.0, 0.0, 0.0};
        if (dt > 0.0) {
            dp.x = vel.x * dt;
            dp.y = vel.y * dt;
            dp.z = vel.z * dt;
        }

        double pn_xy = prev_stop_mode ? pos_noise_xy_stop_ : pos_noise_xy_move_;
        double pn_z = prev_stop_mode ? pos_noise_z_stop_ : pos_noise_z_move_;
        dp.x += normal(pn_xy);
        dp.y += normal(pn_xy);
        dp.z += normal(pn_z);

        pos.x += dp.x;
        pos.y += dp.y;
        pos.z += dp.z;

        double vn_xy = prev_stop_mode ? vel_noise_xy_stop_ : vel_noise_xy_move_;
        double vn_z = prev_stop_mode ? vel_noise_z_stop_ : vel_noise_z_move_;
        vel.x += normal(vn_xy);
        vel.y += normal(vn_xy);
        vel.z += normal(vn_z);

        pos.z = clamp(pos.z, z_min_, z_max_);
        delta_pos_[i] = dp;
    }

    bool stopped_now = (s_norm < s_thr_);
    Vec3 est_pos{0.0, 0.0, 0.0};
    Vec3 est_vel{0.0, 0.0, 0.0};
    double ess_val = 0.0;
    double cond_est = 0.0;

    bool warm = (sample_count_ >= 5);

    if (stopped_now) {
        double invP = 1.0 / static_cast<double>(Pn);
        for (size_t i = 0; i < Pn; ++i) {
            weights_[i] = 0.995 * weights_[i] + 0.005 * invP;
        }
        double sw = std::accumulate(weights_.begin(), weights_.end(), 0.0);
        if (sw <= 0.0 || !std::isfinite(sw)) {
            std::fill(weights_.begin(), weights_.end(), invP);
        } else {
            for (size_t i = 0; i < Pn; ++i) weights_[i] /= sw;
        }

        for (size_t i = 0; i < Pn; ++i) {
            double w = weights_[i];
            const Vec3& p = particles_pos_[i];
            const Vec3& v = particles_vel_[i];
            est_pos.x += w * p.x;
            est_pos.y += w * p.y;
            est_pos.z += w * p.z;
            est_vel.x += w * v.x;
            est_vel.y += w * v.y;
            est_vel.z += w * v.z;
        }

        ess_val = ess();
        cond_est = cond_at_pos(est_pos);

        double essn = clamp(ess_val / static_cast<double>(Pn), 0.0, 1.0);
        double cond_pen = 1.0;
        if (cond_thr_ > 0.0 && cond_est > cond_thr_) {
            double ratio = cond_est / cond_thr_;
            if (ratio < 1.0) ratio = 1.0;
            cond_pen = 1.0 / (1.0 + safe_sqrt(ratio - 1.0));
        }
        double confidence = clamp(0.3 * essn * cond_pen, 0.0, 1.0);

        set_output_fields(out, est_pos.x, est_pos.y, confidence, warm, true);
    } else {
        double max_logw = -std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < Pn; ++i) {
            const Vec3& pos = particles_pos_[i];
            const Vec3& v_prev = particles_vel_[i];

            double h00 = 0.0, h01 = 0.0, h02 = 0.0, h11 = 0.0, h12 = 0.0, h22 = 0.0;
            double hs0 = 0.0, hs1 = 0.0, hs2 = 0.0;

            build_HtH_HtS(pos.x, pos.y, pos.z, s_vec, h00, h01, h02, h11, h12, h22, hs0, hs1, hs2);

            double J_obs = 0.0;
            double cond_obs = 0.0;
            Vec3 v = solve_v_with_prior(h00, h01, h02, h11, h12, h22, hs0, hs1, hs2, v_prev, J_obs, cond_obs, s2);
            v_star_[i] = v;

            double J_kin = 0.0;
            if (has_prev_sample_ && dt > 0.0) {
                const Vec3& dp = delta_pos_[i];
                double ex = dp.x - v.x * dt;
                double ey = dp.y - v.y * dt;
                double ez = dp.z - v.z * dt;
                J_kin = (ex * ex + ey * ey + ez * ez) * inv_sigma_kin2_;
            }

            double vv = v.x * v.x + v.y * v.y + v.z * v.z;
            double vvyz = v.y * v.y + v.z * v.z;

            double denom_sigma = sigma_ * sigma_ + kEps;
            double J = (J_obs / denom_sigma) + beta_kin_ * J_kin + vpen_ * vv + vpen_yz_ * vvyz;

            double sigma_eff = sigma_;
            if (cond_obs > cond_thr_ && cond_thr_ > 0.0) {
                double ratio = cond_obs / cond_thr_;
                if (ratio < 1.0) ratio = 1.0;
                sigma_eff = sigma_ * (1.0 + cond_gain_ * safe_sqrt(ratio));
            }
            sigma_eff = std::max(sigma_eff, 1e-12);

            double lw = -0.5 * J / (sigma_eff * sigma_eff);
            if (!std::isfinite(lw)) lw = -1e30;
            logw_[i] = lw;
            if (lw > max_logw) max_logw = lw;
        }

        double sumw = 0.0;
        for (size_t i = 0; i < Pn; ++i) {
            double w = std::exp(logw_[i] - max_logw);
            if (!std::isfinite(w)) w = 0.0;
            weights_[i] = w;
            sumw += w;
        }

        double invP = 1.0 / static_cast<double>(Pn);
        if (sumw <= 0.0 || !std::isfinite(sumw)) {
            std::fill(weights_.begin(), weights_.end(), invP);
        } else {
            for (size_t i = 0; i < Pn; ++i) weights_[i] /= sumw;
        }

        for (size_t i = 0; i < Pn; ++i) {
            Vec3& vcur = particles_vel_[i];
            const Vec3& vnew = v_star_[i];
            vcur.x = (1.0 - vel_mix_) * vcur.x + vel_mix_ * vnew.x;
            vcur.y = (1.0 - vel_mix_) * vcur.y + vel_mix_ * vnew.y;
            vcur.z = (1.0 - vel_mix_) * vcur.z + vel_mix_ * vnew.z;
        }

        for (size_t i = 0; i < Pn; ++i) {
            double w = weights_[i];
            const Vec3& p = particles_pos_[i];
            const Vec3& v = particles_vel_[i];
            est_pos.x += w * p.x;
            est_pos.y += w * p.y;
            est_pos.z += w * p.z;
            est_vel.x += w * v.x;
            est_vel.y += w * v.y;
            est_vel.z += w * v.z;
        }

        ess_val = ess();
        cond_est = cond_at_pos(est_pos);

        if (ess_val < static_cast<double>(Pn) * ess_ratio_) {
            apply_resample_and_jitter(est_pos, est_vel, prev_stop_mode);
            ess_val = ess();
            cond_est = cond_at_pos(est_pos);
        }

        double gate = 0.0;
        if (s_norm > s_thr_) {
            gate = (s_norm - s_thr_) / (s_thr_ + 1e-12);
            gate = clamp(gate, 0.0, 1.0);
        }

        double essn = clamp(ess_val / static_cast<double>(Pn), 0.0, 1.0);

        double cond_pen = 1.0;
        if (cond_thr_ > 0.0 && cond_est > cond_thr_) {
            double ratio = cond_est / cond_thr_;
            if (ratio < 1.0) ratio = 1.0;
            cond_pen = 1.0 / (1.0 + safe_sqrt(ratio - 1.0));
        }

        double confidence = clamp(essn * gate * cond_pen, 0.0, 1.0);
        set_output_fields(out, est_pos.x, est_pos.y, confidence, warm, false);
    }

    prev_t_ns_ = t_ns;
    for (int ch = 0; ch < kN; ++ch) prev_sample_[ch] = sample[ch];
    has_prev_sample_ = true;
    prev_s_norm_ = s_norm;
    prev_s_thr_ = s_thr_;
    return true;
}

double PF_LA_znotfixed_16x1::clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

double PF_LA_znotfixed_16x1::safe_sqrt(double v) {
    if (v <= 0.0) return 0.0;
    return std::sqrt(v);
}

double PF_LA_znotfixed_16x1::uniform01() {
    double u = uni_(rng_);
    if (u <= 0.0) u = 1e-12;
    if (u >= 1.0) u = 1.0 - 1e-12;
    return u;
}

double PF_LA_znotfixed_16x1::normal01() {
    if (has_spare_normal_) {
        has_spare_normal_ = false;
        return spare_normal_;
    }
    double u1 = uniform01();
    double u2 = uniform01();
    double r = std::sqrt(-2.0 * std::log(u1));
    double theta = 6.2831853071795864769 * u2;
    double z0 = r * std::cos(theta);
    double z1 = r * std::sin(theta);
    spare_normal_ = z1;
    has_spare_normal_ = true;
    return z0;
}

double PF_LA_znotfixed_16x1::normal(double stddev) {
    if (stddev <= 0.0) return 0.0;
    return normal01() * stddev;
}

double PF_LA_znotfixed_16x1::normal(double mean, double stddev) {
    return mean + normal(stddev);
}

void PF_LA_znotfixed_16x1::ensure_storage() {
    if (P_ < 100) P_ = 100;
    size_t Pn = static_cast<size_t>(P_);

    particles_pos_.resize(Pn);
    particles_vel_.resize(Pn);
    weights_.resize(Pn);
    v_star_.resize(Pn);
    delta_pos_.resize(Pn);
    logw_.resize(Pn);
    resample_idx_.resize(Pn);
    cdf_.resize(Pn);
    perm_idx_.resize(Pn);
    tmp_pos_.resize(Pn);
    tmp_vel_.resize(Pn);

    for (size_t i = 0; i < Pn; ++i) perm_idx_[i] = i;
}

void PF_LA_znotfixed_16x1::init_particles() {
    const size_t Pn = particles_pos_.size();
    if (Pn == 0) return;

    for (size_t i = 0; i < Pn; ++i) {
        Vec3 p;
        p.x = normal(0.0, 15.0);
        p.y = normal(0.0, 15.0);
        p.z = normal(18.0, 8.0);
        p.z = clamp(p.z, z_min_, z_max_);
        particles_pos_[i] = p;

        Vec3 v;
        v.x = normal(0.0, 120.0);
        v.y = normal(0.0, 120.0);
        v.z = normal(0.0, 120.0);
        particles_vel_[i] = v;
    }

    double invP = 1.0 / static_cast<double>(Pn);
    std::fill(weights_.begin(), weights_.end(), invP);
}

void PF_LA_znotfixed_16x1::push_snorm(double sn) {
    if (!std::isfinite(sn)) sn = 0.0;
    if (s_norm_buf_fill_ < kMedianBuf) {
        s_norm_buf_[s_norm_buf_fill_] = sn;
        ++s_norm_buf_fill_;
    } else {
        s_norm_buf_[s_norm_buf_head_] = sn;
        s_norm_buf_head_ = (s_norm_buf_head_ + 1) % kMedianBuf;
        s_norm_buf_fill_ = kMedianBuf;
    }
}

void PF_LA_znotfixed_16x1::update_median_and_scales() {
    const size_t n = s_norm_buf_fill_;
    if (n == 0) {
        median_s_norm_ = 0.0;
        sigma_ = 1.0;
        s_thr_ = 0.0;
        return;
    }

    for (size_t i = 0; i < n; ++i) {
        s_norm_scratch_[i] = s_norm_buf_[i];
    }

    const size_t mid = n / 2;
    auto b = s_norm_scratch_.begin();
    auto m = b + static_cast<std::ptrdiff_t>(mid);
    auto e = b + static_cast<std::ptrdiff_t>(n);

    std::nth_element(b, m, e);
    double med_hi = *m;

    double med = med_hi;
    if ((n % 2) == 0) {
        double med_lo = *std::max_element(b, m);
        med = 0.5 * (med_lo + med_hi);
    }

    if (!std::isfinite(med) || med < 0.0) med = 0.0;
    median_s_norm_ = med;

    double denom = std::sqrt(static_cast<double>(kUsed));
    sigma_ = sigma_scale_ * (median_s_norm_ / denom + 1e-12);
    if (!std::isfinite(sigma_) || sigma_ < 1e-12) sigma_ = 1e-12;

    s_thr_ = gate_ratio_ * median_s_norm_;
    if (!std::isfinite(s_thr_) || s_thr_ < 0.0) s_thr_ = 0.0;
}

PF_LA_znotfixed_16x1::Vec3 PF_LA_znotfixed_16x1::solve_cholesky3(
    double a00, double a01, double a02,
    double a11, double a12,
    double a22,
    double b0, double b1, double b2) {
    a00 = std::max(a00, 1e-18);
    a11 = std::max(a11, 1e-18);
    a22 = std::max(a22, 1e-18);

    double l00 = safe_sqrt(a00);
    if (l00 < 1e-18) return Vec3{0.0, 0.0, 0.0};

    double l10 = a01 / l00;
    double l20 = a02 / l00;

    double t11 = a11 - l10 * l10;
    double l11 = safe_sqrt(t11);
    if (l11 < 1e-18) l11 = 1e-18;

    double l21 = (a12 - l20 * l10) / l11;

    double t22 = a22 - l20 * l20 - l21 * l21;
    double l22 = safe_sqrt(t22);
    if (l22 < 1e-18) l22 = 1e-18;

    double y0 = b0 / l00;
    double y1 = (b1 - l10 * y0) / l11;
    double y2 = (b2 - l20 * y0 - l21 * y1) / l22;

    double x2 = y2 / l22;
    double x1 = (y1 - l21 * x2) / l11;
    double x0 = (y0 - l10 * x1 - l20 * x2) / l00;

    if (!std::isfinite(x0)) x0 = 0.0;
    if (!std::isfinite(x1)) x1 = 0.0;
    if (!std::isfinite(x2)) x2 = 0.0;

    return Vec3{x0, x1, x2};
}

double PF_LA_znotfixed_16x1::cond_sym3(
    double a00, double a01, double a02,
    double a11, double a12,
    double a22) {
    double r0 = std::abs(a01) + std::abs(a02);
    double r1 = std::abs(a01) + std::abs(a12);
    double r2 = std::abs(a02) + std::abs(a12);

    double lmax = std::max({a00 + r0, a11 + r1, a22 + r2});
    double lmin = std::min({a00 - r0, a11 - r1, a22 - r2});

    lmin = std::max(lmin, 1e-18);
    if (!std::isfinite(lmax) || lmax < 1e-18) lmax = 1e-18;
    return lmax / lmin;
}

void PF_LA_znotfixed_16x1::build_HtH_HtS(
    double x, double y, double z,
    const std::array<double, kUsed>& s,
    double& h00, double& h01, double& h02,
    double& h11, double& h12, double& h22,
    double& hs0, double& hs1, double& hs2) const {
    double z_eff = z + z0_;
    double Z = alpha_ * z_eff;
    double alpha2 = alpha_ * alpha_;
    double z_eff2 = z_eff * z_eff;

    for (int i = 0; i < kUsed; ++i) {
        double dx = x - xs_u_[i];
        double dy = y - ys_u_[i];

        double r2 = dx * dx + dy * dy + Z * Z;
        double inv_r2 = 1.0 / (r2 + kEps);
        double r2sq = r2 * r2 + kEps;
        double inv_r2sq = 1.0 / r2sq;

        double h0 = kprime_ * (-2.0 * z_eff * dx * inv_r2sq);
        double h1 = kprime_ * (-2.0 * z_eff * dy * inv_r2sq);
        double h2 = kprime_ * (inv_r2 - (2.0 * alpha2 * z_eff2) * inv_r2sq);

        h00 += h0 * h0;
        h01 += h0 * h1;
        h02 += h0 * h2;
        h11 += h1 * h1;
        h12 += h1 * h2;
        h22 += h2 * h2;

        double si = s[i];
        hs0 += h0 * si;
        hs1 += h1 * si;
        hs2 += h2 * si;
    }
}

PF_LA_znotfixed_16x1::Vec3 PF_LA_znotfixed_16x1::solve_v_with_prior(
    double h00, double h01, double h02,
    double h11, double h12, double h22,
    double hs0, double hs1, double hs2,
    const Vec3& v_prev,
    double& J_obs,
    double& cond_obs,
    double s2) const {
    double a00 = h00 + ridge_;
    double a11 = h11 + ridge_;
    double a22 = h22 + ridge_;
    double a01 = h01;
    double a02 = h02;
    double a12 = h12;

    cond_obs = cond_sym3(a00, a01, a02, a11, a12, a22);

    double A00 = a00 + lam_vx_;
    double A11 = a11 + lam_vy_;
    double A22 = a22 + lam_vz_;

    double b0 = hs0 + lam_vx_ * v_prev.x;
    double b1 = hs1 + lam_vy_ * v_prev.y;
    double b2 = hs2 + lam_vz_ * v_prev.z;

    Vec3 v = solve_cholesky3(A00, a01, a02, A11, a12, A22, b0, b1, b2);

    double vT_HtS = v.x * hs0 + v.y * hs1 + v.z * hs2;

    double vT_HtH_v =
        v.x * (h00 * v.x + h01 * v.y + h02 * v.z)
        + v.y * (h01 * v.x + h11 * v.y + h12 * v.z)
        + v.z * (h02 * v.x + h12 * v.y + h22 * v.z);

    double e2 = s2 - 2.0 * vT_HtS + vT_HtH_v;
    if (!std::isfinite(e2)) e2 = 0.0;
    if (e2 < 0.0) e2 = 0.0;

    double dvx = v.x - v_prev.x;
    double dvy = v.y - v_prev.y;
    double dvz = v.z - v_prev.z;

    double dvLamdv = lam_vx_ * dvx * dvx + lam_vy_ * dvy * dvy + lam_vz_ * dvz * dvz;
    if (!std::isfinite(dvLamdv)) dvLamdv = 0.0;

    J_obs = e2 + dvLamdv;
    if (!std::isfinite(J_obs)) J_obs = 0.0;

    return v;
}

double PF_LA_znotfixed_16x1::cond_at_pos(const Vec3& pos) const {
    double h00 = 0.0, h01 = 0.0, h02 = 0.0, h11 = 0.0, h12 = 0.0, h22 = 0.0;
    double hs0 = 0.0, hs1 = 0.0, hs2 = 0.0;
    std::array<double, kUsed> s{};
    s.fill(0.0);
    build_HtH_HtS(pos.x, pos.y, pos.z, s, h00, h01, h02, h11, h12, h22, hs0, hs1, hs2);
    return cond_sym3(h00 + ridge_, h01, h02, h11 + ridge_, h12, h22 + ridge_);
}

double PF_LA_znotfixed_16x1::ess() const {
    double sumsq = 0.0;
    for (double w : weights_) sumsq += w * w;
    if (!std::isfinite(sumsq) || sumsq <= 0.0) return 0.0;
    return 1.0 / (sumsq + 1e-18);
}

void PF_LA_znotfixed_16x1::systematic_resample() {
    const size_t Pn = weights_.size();
    if (Pn == 0) return;

    cdf_[0] = weights_[0];
    for (size_t i = 1; i < Pn; ++i) cdf_[i] = cdf_[i - 1] + weights_[i];

    double total = cdf_[Pn - 1];
    if (total <= 0.0 || !std::isfinite(total)) {
        for (size_t i = 0; i < Pn; ++i) resample_idx_[i] = i;
        return;
    }
    for (size_t i = 0; i < Pn; ++i) cdf_[i] /= total;
    cdf_[Pn - 1] = 1.0;

    double invP = 1.0 / static_cast<double>(Pn);
    double u0 = uniform01() * invP;

    size_t idx = 0;
    for (size_t j = 0; j < Pn; ++j) {
        double u = u0 + static_cast<double>(j) * invP;
        while (idx + 1 < Pn && u > cdf_[idx]) ++idx;
        resample_idx_[j] = idx;
    }
}

void PF_LA_znotfixed_16x1::apply_resample_and_jitter(const Vec3& est_pos, const Vec3& est_vel, bool prev_stop_mode) {
    const size_t Pn = weights_.size();
    if (Pn == 0) return;

    systematic_resample();

    for (size_t i = 0; i < Pn; ++i) {
        size_t j = resample_idx_[i];
        tmp_pos_[i] = particles_pos_[j];
        tmp_vel_[i] = particles_vel_[j];
    }

    particles_pos_.swap(tmp_pos_);
    particles_vel_.swap(tmp_vel_);

    double invP = 1.0 / static_cast<double>(Pn);
    std::fill(weights_.begin(), weights_.end(), invP);

    for (size_t i = 0; i < Pn; ++i) {
        particles_pos_[i].x += normal(jitter_xy_);
        particles_pos_[i].y += normal(jitter_xy_);
        particles_pos_[i].z += normal(jitter_z_);
        particles_pos_[i].z = clamp(particles_pos_[i].z, z_min_, z_max_);
    }

    if (!prev_stop_mode) {
        int R = static_cast<int>(std::llround(respawn_ratio_move_ * static_cast<double>(Pn)));
        if (R < 0) R = 0;
        if (R > static_cast<int>(Pn)) R = static_cast<int>(Pn);

        if (R > 0) {
            for (size_t i = 0; i < Pn; ++i) perm_idx_[i] = i;
            std::shuffle(perm_idx_.begin(), perm_idx_.end(), rng_);

            for (int r = 0; r < R; ++r) {
                size_t idx = perm_idx_[static_cast<size_t>(r)];

                particles_pos_[idx].x = normal(est_pos.x, 60.0);
                particles_pos_[idx].y = normal(est_pos.y, 60.0);
                particles_pos_[idx].z = normal(est_pos.z, 25.0);
                particles_pos_[idx].z = clamp(particles_pos_[idx].z, z_min_, z_max_);

                particles_vel_[idx].x = normal(est_vel.x, 200.0);
                particles_vel_[idx].y = normal(est_vel.y, 200.0);
                particles_vel_[idx].z = normal(est_vel.z, 200.0);
            }
        }
    }
}

void PF_LA_znotfixed_16x1::set_output_fields(hub::pt::Output& out, double x, double y, double confidence, bool valid, bool quiet) {
    out.x = static_cast<decltype(out.x)>(x);
    out.y = static_cast<decltype(out.y)>(y);
    out.confidence = static_cast<decltype(out.confidence)>(clamp(confidence, 0.0, 1.0));
    out.valid = valid;
    out.quiet = quiet;
}

HUB_PT_REGISTER_ALGORITHM(PF_LA_znotfixed_16x1)

}