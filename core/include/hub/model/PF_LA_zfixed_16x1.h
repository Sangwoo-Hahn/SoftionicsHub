#pragma once
#include "hub/model/PositionTrackingRegistry.h"
#include <array>
#include <vector>
#include <string>
#include <cstdint>
#include <random>

namespace hub::pt {

class PF_LA_zfixed_16x1 final : public IAlgorithm {
public:
    PF_LA_zfixed_16x1();
    const std::string& id() const override;
    int N() const override;
    int M() const override;
    std::vector<ParamDesc> params() const override;
    std::vector<double> defaults() const override;
    void set_params(const std::vector<double>& values) override;
    void reset() override;
    bool push_sample(uint64_t t_ns, const std::vector<float>& sample, Output& out) override;

private:
    struct Vec3 { double x; double y; double z; };
    struct Mat3 { double m00, m01, m02, m10, m11, m12, m20, m21, m22; };
    struct Sym3 { double a00, a01, a02, a11, a12, a22; };

    struct Params {
        int particles;
        uint64_t seed;
        double z_fixed_m;
        double k_dv_s;
        double deriv_tau_s;
        double pos_noise_m;
        double vel_noise_mps;
        double jitter_m;
        double ridge;
        double lam_vx;
        double lam_vy;
        double lam_vz;
        double vel_mix;
        double cond_thr;
        double cond_gain;
        double ess_ratio;
        double respawn_ratio;
        double respawn_info_min;
        double p_info;
        double sigma_gain;
        double lam_gain;
        double w_floor_max;
        double vel_damp_max;
        double x_refine_on;
        double x_span_m;
        double x_step_m;
        double x_refine_gain;
        double x_refine_info_min;
        double dx_clamp_m;
    };

    static constexpr int kN = 16;
    static constexpr int kM = 1;
    static constexpr int kUsed = 12;

    static const std::array<int, kUsed>& used_ids();
    static std::array<double, kN> default_sensor_x();
    static std::array<double, kN> default_sensor_y();

    static Vec3 v_add(const Vec3& a, const Vec3& b);
    static Vec3 v_sub(const Vec3& a, const Vec3& b);
    static Vec3 v_scale(const Vec3& a, double s);
    static double v_dot(const Vec3& a, const Vec3& b);
    static Vec3 v_cross(const Vec3& a, const Vec3& b);
    static double v_norm(const Vec3& a);
    static Vec3 v_normalize(const Vec3& a, double eps);

    static Mat3 mat_identity();
    static Mat3 mat_transpose(const Mat3& A);
    static Mat3 mat_mul(const Mat3& A, const Mat3& B);
    static Vec3 mat_mul_vec(const Mat3& A, const Vec3& v);
    static Vec3 row_mul_mat(const Vec3& row, const Mat3& A);

    static Mat3 rot_a_to_b(const Vec3& a, const Vec3& b);
    static Vec3 rot_log(const Mat3& R);
    static Mat3 rot_exp(const Vec3& r);
    static Mat3 smooth_rot(const Mat3& R_prev, const Mat3& R_new, double gamma);

    static Sym3 sym_zero();
    static Sym3 sym_add_diag(const Sym3& A, double d0, double d1, double d2);
    static Sym3 sym_outer_add(const Sym3& A, const Vec3& h);
    static double sym_det(const Sym3& A);
    static void sym_eigenvalues(const Sym3& A, double& l0, double& l1, double& l2);
    static double sym_cond_2norm(const Sym3& A);
    static bool solve_linear3(const Sym3& A, const Vec3& b, Vec3& x);

    Vec3 build_H_row(double x, double y, double z, int sensor_id) const;
    bool solve_vprime_and_stats(double x, double y, double z, const Mat3& RT, const std::array<double, kUsed>& s, const Vec3& vprime_prev, const Vec3& lam_diag, double ridge, Vec3& vprime, double& J_obs, double& condAobs) const;
    bool refine_x_1d(double x0_guess, double y, double z, const std::array<double, kUsed>& s, const Mat3& R, const Mat3& RT, double sigma_eff, const Vec3& lam_diag, const Vec3& v_prev_world, double& x_best, Vec3& v_best_world) const;
    void systematic_resample();
    void ensure_alloc();

    Params p_;
    std::array<double, kN> sx_;
    std::array<double, kN> sy_;

    std::vector<Vec3> pos_;
    std::vector<Vec3> vel_;
    std::vector<Vec3> pos_tmp_;
    std::vector<Vec3> vel_tmp_;
    std::vector<Vec3> v_star_;
    std::vector<double> weights_;
    std::vector<double> logw_;
    std::vector<int> idx_buf_;

    std::array<double, kN> v_prev_;
    std::array<double, kN> dvdt_;
    bool have_prev_;
    uint64_t prev_t_ns_;
    uint64_t step_count_;

    double sref_ema_;
    Mat3 R_;
    Vec3 q_dir_;

    std::mt19937_64 rng_;
    std::normal_distribution<double> norm01_;
    std::uniform_real_distribution<double> unif01_;
};

}