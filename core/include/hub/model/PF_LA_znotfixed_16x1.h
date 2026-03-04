#pragma once
#include "hub/model/PositionTrackingRegistry.h"
#include <array>
#include <vector>
#include <string>
#include <cstdint>
#include <random>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>

namespace hub::pt {

class PF_LA_znotfixed_16x1 final : public IAlgorithm {
public:
    PF_LA_znotfixed_16x1();
    ~PF_LA_znotfixed_16x1() override = default;

    const std::string& id() const override;
    int N() const override;
    int M() const override;

    std::vector<hub::pt::ParamDesc> params() const override;
    std::vector<double> defaults() const override;
    void set_params(const std::vector<double>& values) override;

    void reset() override;

    bool push_sample(uint64_t t_ns, const std::vector<float>& sample, hub::pt::Output& out) override;

private:
    struct Vec3 {
        double x;
        double y;
        double z;
    };

    static constexpr int kN = 16;
    static constexpr int kM = 1;
    static constexpr int kUsed = 12;
    static constexpr size_t kMedianBuf = 256;
    static constexpr uint64_t kMedianUpdateInterval = 32;
    static constexpr double kEps = 1e-18;

    std::array<int, kUsed> used_sids_;
    std::array<double, kUsed> xs_u_;
    std::array<double, kUsed> ys_u_;

    double k_dv_;
    double deriv_window_ms_;
    double kprime_;
    double alpha_;
    double z0_;

    int P_;

    double ess_ratio_;
    double respawn_ratio_move_;

    double pos_noise_xy_move_;
    double pos_noise_z_move_;
    double vel_noise_xy_move_;
    double vel_noise_z_move_;

    double pos_noise_xy_stop_;
    double pos_noise_z_stop_;
    double vel_noise_xy_stop_;
    double vel_noise_z_stop_;

    double stop_damp_;
    double jitter_xy_;
    double jitter_z_;

    double z_min_;
    double z_max_;

    double ridge_;
    double lam_vx_;
    double lam_vy_;
    double lam_vz_;

    double beta_kin_;
    double sigma_kin_;
    double inv_sigma_kin2_;

    double sigma_scale_;
    double gate_ratio_;

    double vpen_;
    double vpen_yz_;

    double cond_thr_;
    double cond_gain_;

    double vel_mix_;

    double sigma_;
    double s_thr_;
    double median_s_norm_;

    std::vector<Vec3> particles_pos_;
    std::vector<Vec3> particles_vel_;
    std::vector<double> weights_;
    std::vector<Vec3> v_star_;
    std::vector<Vec3> delta_pos_;
    std::vector<double> logw_;
    std::vector<size_t> resample_idx_;
    std::vector<double> cdf_;
    std::vector<size_t> perm_idx_;
    std::vector<Vec3> tmp_pos_;
    std::vector<Vec3> tmp_vel_;

    bool has_prev_sample_;
    std::array<float, kN> prev_sample_;
    std::array<double, kN> dVdt_smooth_;
    uint64_t prev_t_ns_;
    double prev_s_norm_;
    double prev_s_thr_;
    uint64_t sample_count_;

    std::array<double, kMedianBuf> s_norm_buf_;
    std::array<double, kMedianBuf> s_norm_scratch_;
    size_t s_norm_buf_fill_;
    size_t s_norm_buf_head_;

    std::mt19937 rng_;
    std::uniform_real_distribution<double> uni_;
    bool has_spare_normal_;
    double spare_normal_;

    static double clamp(double v, double lo, double hi);
    static double safe_sqrt(double v);

    double uniform01();
    double normal01();
    double normal(double stddev);
    double normal(double mean, double stddev);

    void ensure_storage();
    void init_particles();

    void push_snorm(double sn);
    void update_median_and_scales();

    static Vec3 solve_cholesky3(
        double a00, double a01, double a02,
        double a11, double a12,
        double a22,
        double b0, double b1, double b2);

    static double cond_sym3(
        double a00, double a01, double a02,
        double a11, double a12,
        double a22);

    void build_HtH_HtS(
        double x, double y, double z,
        const std::array<double, kUsed>& s,
        double& h00, double& h01, double& h02,
        double& h11, double& h12, double& h22,
        double& hs0, double& hs1, double& hs2) const;

    Vec3 solve_v_with_prior(
        double h00, double h01, double h02,
        double h11, double h12, double h22,
        double hs0, double hs1, double hs2,
        const Vec3& v_prev,
        double& J_obs,
        double& cond_obs,
        double s2) const;

    double cond_at_pos(const Vec3& pos) const;
    double ess() const;

    void systematic_resample();
    void apply_resample_and_jitter(const Vec3& est_pos, const Vec3& est_vel, bool prev_stop_mode);

    static void set_output_fields(hub::pt::Output& out, double x, double y, double confidence, bool valid, bool quiet);
};

}