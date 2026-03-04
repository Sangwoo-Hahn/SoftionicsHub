#include "hub/model/PF_LA_zfixed_16x1.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

namespace hub::pt {

static const std::string kAlgoId = "PF_LA_zfixed_16x1";

static constexpr double kKprime = -2.663;
static constexpr double kAlpha = 1.250;
static constexpr double kZ0 = 2.0e-3;
static constexpr double kEps = 1e-18;

static constexpr double kInfoMin = 0.02;
static constexpr double kInfoMax = 1.0;

static constexpr double kSigmaBaseFactor = 1.05;
static constexpr double kSRefTauS = 2.0;

static constexpr double kVelDampPow = 1.6;
static constexpr double kVelNoiseScalePow = 1.2;

static constexpr double kVpen = 3e-6;
static constexpr double kVpenYZ = 2e-5;

static constexpr double kRotGamma = 0.10;
static constexpr double kRotGammaMin = 0.01;
static constexpr double kRotGammaMax = 0.18;
static constexpr double kRotGammaPow = 1.4;

static constexpr double kBetaDirGain = 0.12;
static constexpr double kBetaDirPow = 1.6;

static constexpr double kXRefineGainPow = 1.3;

static constexpr double kInitPosStd = 15.0e-3;
static constexpr double kInitVelStdXY = 120.0e-3;
static constexpr double kInitVelStdZ = 60.0e-3;

static constexpr double kRespawnPosStd = 60.0e-3;
static constexpr double kRespawnVelStdXY = 200.0e-3;
static constexpr double kRespawnVelStdZ = 120.0e-3;

static constexpr double kVelScaleMin = 0.12;

template <typename T>
static auto set_member_z(T& o, double v, int) -> decltype(o.z = v, void()) { o.z = v; }

template <typename T>
static void set_member_z(T&, double, long) {}

template <typename T>
static auto set_member_vx(T& o, double v, int) -> decltype(o.vx = v, void()) { o.vx = v; }

template <typename T>
static void set_member_vx(T&, double, long) {}

template <typename T>
static auto set_member_vy(T& o, double v, int) -> decltype(o.vy = v, void()) { o.vy = v; }

template <typename T>
static void set_member_vy(T&, double, long) {}

template <typename T>
static auto set_member_vz(T& o, double v, int) -> decltype(o.vz = v, void()) { o.vz = v; }

template <typename T>
static void set_member_vz(T&, double, long) {}

static double clampd(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint64_t clampu64(uint64_t v, uint64_t lo, uint64_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

const std::array<int, PF_LA_zfixed_16x1::kUsed>& PF_LA_zfixed_16x1::used_ids() {
    static const std::array<int, kUsed> ids = { 0, 1, 2, 4, 6, 7, 8, 9, 10, 12, 14, 15 };
    return ids;
}

std::array<double, PF_LA_zfixed_16x1::kN> PF_LA_zfixed_16x1::default_sensor_x() {
    constexpr double d = 19.1e-3;
    return {
        -1.5 * d,
         0.5 * d,
         1.5 * d,
         0.5 * d,
         1.5 * d,
         0.5 * d,
         1.5 * d,
         0.5 * d,
         1.5 * d,
        -0.5 * d,
        -1.5 * d,
        -0.5 * d,
        -1.5 * d,
        -0.5 * d,
        -1.5 * d,
        -0.5 * d
    };
}

std::array<double, PF_LA_zfixed_16x1::kN> PF_LA_zfixed_16x1::default_sensor_y() {
    constexpr double d = 19.1e-3;
    return {
        -1.5 * d,
        -1.5 * d,
        -1.5 * d,
        -0.5 * d,
        -0.5 * d,
         0.5 * d,
         0.5 * d,
         1.5 * d,
         1.5 * d,
         1.5 * d,
         1.5 * d,
         0.5 * d,
         0.5 * d,
        -0.5 * d,
        -0.5 * d,
        -1.5 * d
    };
}

PF_LA_zfixed_16x1::Vec3 PF_LA_zfixed_16x1::v_add(const Vec3& a, const Vec3& b) {
    return Vec3{ a.x + b.x, a.y + b.y, a.z + b.z };
}

PF_LA_zfixed_16x1::Vec3 PF_LA_zfixed_16x1::v_sub(const Vec3& a, const Vec3& b) {
    return Vec3{ a.x - b.x, a.y - b.y, a.z - b.z };
}

PF_LA_zfixed_16x1::Vec3 PF_LA_zfixed_16x1::v_scale(const Vec3& a, double s) {
    return Vec3{ a.x * s, a.y * s, a.z * s };
}

double PF_LA_zfixed_16x1::v_dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

PF_LA_zfixed_16x1::Vec3 PF_LA_zfixed_16x1::v_cross(const Vec3& a, const Vec3& b) {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

double PF_LA_zfixed_16x1::v_norm(const Vec3& a) {
    return std::sqrt(v_dot(a, a));
}

PF_LA_zfixed_16x1::Vec3 PF_LA_zfixed_16x1::v_normalize(const Vec3& a, double eps) {
    double n = v_norm(a);
    if (!(n > eps)) return Vec3{ 0.0, 0.0, 0.0 };
    double inv = 1.0 / n;
    return Vec3{ a.x * inv, a.y * inv, a.z * inv };
}

PF_LA_zfixed_16x1::Mat3 PF_LA_zfixed_16x1::mat_identity() {
    return Mat3{ 1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0 };
}

PF_LA_zfixed_16x1::Mat3 PF_LA_zfixed_16x1::mat_transpose(const Mat3& A) {
    return Mat3{ A.m00, A.m10, A.m20,
                 A.m01, A.m11, A.m21,
                 A.m02, A.m12, A.m22 };
}

PF_LA_zfixed_16x1::Mat3 PF_LA_zfixed_16x1::mat_mul(const Mat3& A, const Mat3& B) {
    Mat3 C;
    C.m00 = A.m00 * B.m00 + A.m01 * B.m10 + A.m02 * B.m20;
    C.m01 = A.m00 * B.m01 + A.m01 * B.m11 + A.m02 * B.m21;
    C.m02 = A.m00 * B.m02 + A.m01 * B.m12 + A.m02 * B.m22;

    C.m10 = A.m10 * B.m00 + A.m11 * B.m10 + A.m12 * B.m20;
    C.m11 = A.m10 * B.m01 + A.m11 * B.m11 + A.m12 * B.m21;
    C.m12 = A.m10 * B.m02 + A.m11 * B.m12 + A.m12 * B.m22;

    C.m20 = A.m20 * B.m00 + A.m21 * B.m10 + A.m22 * B.m20;
    C.m21 = A.m20 * B.m01 + A.m21 * B.m11 + A.m22 * B.m21;
    C.m22 = A.m20 * B.m02 + A.m21 * B.m12 + A.m22 * B.m22;
    return C;
}

PF_LA_zfixed_16x1::Vec3 PF_LA_zfixed_16x1::mat_mul_vec(const Mat3& A, const Vec3& v) {
    return Vec3{
        A.m00 * v.x + A.m01 * v.y + A.m02 * v.z,
        A.m10 * v.x + A.m11 * v.y + A.m12 * v.z,
        A.m20 * v.x + A.m21 * v.y + A.m22 * v.z
    };
}

PF_LA_zfixed_16x1::Vec3 PF_LA_zfixed_16x1::row_mul_mat(const Vec3& row, const Mat3& A) {
    return Vec3{
        row.x * A.m00 + row.y * A.m10 + row.z * A.m20,
        row.x * A.m01 + row.y * A.m11 + row.z * A.m21,
        row.x * A.m02 + row.y * A.m12 + row.z * A.m22
    };
}

PF_LA_zfixed_16x1::Mat3 PF_LA_zfixed_16x1::rot_a_to_b(const Vec3& a_in, const Vec3& b_in) {
    Vec3 a = v_normalize(a_in, 1e-12);
    Vec3 b = v_normalize(b_in, 1e-12);
    if (v_norm(a) < 1e-12 || v_norm(b) < 1e-12) return mat_identity();

    Vec3 v = v_cross(a, b);
    double s = v_norm(v);
    double c = v_dot(a, b);

    if (s < 1e-12) {
        if (c > 0.0) return mat_identity();
        Vec3 axis{ 1.0, 0.0, 0.0 };
        if (std::abs(a.x) > 0.9) axis = Vec3{ 0.0, 1.0, 0.0 };
        double proj = v_dot(axis, a);
        axis = v_sub(axis, v_scale(a, proj));
        axis = v_normalize(axis, 1e-12);
        Mat3 K{
            0.0, -axis.z, axis.y,
            axis.z, 0.0, -axis.x,
            -axis.y, axis.x, 0.0
        };
        Mat3 K2 = mat_mul(K, K);
        Mat3 I = mat_identity();
        return Mat3{
            I.m00 + 2.0 * K2.m00, I.m01 + 2.0 * K2.m01, I.m02 + 2.0 * K2.m02,
            I.m10 + 2.0 * K2.m10, I.m11 + 2.0 * K2.m11, I.m12 + 2.0 * K2.m12,
            I.m20 + 2.0 * K2.m20, I.m21 + 2.0 * K2.m21, I.m22 + 2.0 * K2.m22
        };
    }

    Vec3 k = v_scale(v, 1.0 / s);
    Mat3 K{
        0.0, -k.z, k.y,
        k.z, 0.0, -k.x,
        -k.y, k.x, 0.0
    };
    Mat3 K2 = mat_mul(K, K);
    Mat3 I = mat_identity();

    return Mat3{
        I.m00 + K.m00 * s + K2.m00 * (1.0 - c), I.m01 + K.m01 * s + K2.m01 * (1.0 - c), I.m02 + K.m02 * s + K2.m02 * (1.0 - c),
        I.m10 + K.m10 * s + K2.m10 * (1.0 - c), I.m11 + K.m11 * s + K2.m11 * (1.0 - c), I.m12 + K.m12 * s + K2.m12 * (1.0 - c),
        I.m20 + K.m20 * s + K2.m20 * (1.0 - c), I.m21 + K.m21 * s + K2.m21 * (1.0 - c), I.m22 + K.m22 * s + K2.m22 * (1.0 - c)
    };
}

PF_LA_zfixed_16x1::Vec3 PF_LA_zfixed_16x1::rot_log(const Mat3& R) {
    double tr = R.m00 + R.m11 + R.m22;
    tr = clampd(tr, -1.0, 3.0);
    double cos_arg = clampd((tr - 1.0) * 0.5, -1.0, 1.0);
    double theta = std::acos(cos_arg);
    if (!(theta > 1e-12)) return Vec3{ 0.0, 0.0, 0.0 };
    double s = std::sin(theta);
    if (!(std::abs(s) > 1e-12)) return Vec3{ 0.0, 0.0, 0.0 };
    double inv = 0.5 / s;
    Vec3 w{
        (R.m21 - R.m12) * inv,
        (R.m02 - R.m20) * inv,
        (R.m10 - R.m01) * inv
    };
    return v_scale(w, theta);
}

PF_LA_zfixed_16x1::Mat3 PF_LA_zfixed_16x1::rot_exp(const Vec3& r) {
    double theta = v_norm(r);
    if (!(theta > 1e-12)) return mat_identity();
    Vec3 k = v_scale(r, 1.0 / theta);
    Mat3 K{
        0.0, -k.z, k.y,
        k.z, 0.0, -k.x,
        -k.y, k.x, 0.0
    };
    Mat3 K2 = mat_mul(K, K);
    Mat3 I = mat_identity();
    double s = std::sin(theta);
    double c = std::cos(theta);
    return Mat3{
        I.m00 + K.m00 * s + K2.m00 * (1.0 - c), I.m01 + K.m01 * s + K2.m01 * (1.0 - c), I.m02 + K.m02 * s + K2.m02 * (1.0 - c),
        I.m10 + K.m10 * s + K2.m10 * (1.0 - c), I.m11 + K.m11 * s + K2.m11 * (1.0 - c), I.m12 + K.m12 * s + K2.m12 * (1.0 - c),
        I.m20 + K.m20 * s + K2.m20 * (1.0 - c), I.m21 + K.m21 * s + K2.m21 * (1.0 - c), I.m22 + K.m22 * s + K2.m22 * (1.0 - c)
    };
}

PF_LA_zfixed_16x1::Mat3 PF_LA_zfixed_16x1::smooth_rot(const Mat3& R_prev, const Mat3& R_new, double gamma) {
    Mat3 Rt_prev = mat_transpose(R_prev);
    Mat3 dR = mat_mul(R_new, Rt_prev);
    Vec3 r = rot_log(dR);
    Mat3 inc = rot_exp(v_scale(r, gamma));
    return mat_mul(inc, R_prev);
}

PF_LA_zfixed_16x1::Sym3 PF_LA_zfixed_16x1::sym_zero() {
    return Sym3{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
}

PF_LA_zfixed_16x1::Sym3 PF_LA_zfixed_16x1::sym_add_diag(const Sym3& A, double d0, double d1, double d2) {
    return Sym3{ A.a00 + d0, A.a01, A.a02, A.a11 + d1, A.a12, A.a22 + d2 };
}

PF_LA_zfixed_16x1::Sym3 PF_LA_zfixed_16x1::sym_outer_add(const Sym3& A, const Vec3& h) {
    return Sym3{
        A.a00 + h.x * h.x,
        A.a01 + h.x * h.y,
        A.a02 + h.x * h.z,
        A.a11 + h.y * h.y,
        A.a12 + h.y * h.z,
        A.a22 + h.z * h.z
    };
}

double PF_LA_zfixed_16x1::sym_det(const Sym3& A) {
    double a00 = A.a00;
    double a01 = A.a01;
    double a02 = A.a02;
    double a11 = A.a11;
    double a12 = A.a12;
    double a22 = A.a22;
    return a00 * (a11 * a22 - a12 * a12) - a01 * (a01 * a22 - a12 * a02) + a02 * (a01 * a12 - a11 * a02);
}

void PF_LA_zfixed_16x1::sym_eigenvalues(const Sym3& A, double& l0, double& l1, double& l2) {
    double p1 = A.a01 * A.a01 + A.a02 * A.a02 + A.a12 * A.a12;
    if (!(p1 > 0.0)) {
        l0 = A.a00;
        l1 = A.a11;
        l2 = A.a22;
    } else {
        double q = (A.a00 + A.a11 + A.a22) / 3.0;
        double a00 = A.a00 - q;
        double a11 = A.a11 - q;
        double a22 = A.a22 - q;
        double p2 = a00 * a00 + a11 * a11 + a22 * a22 + 2.0 * p1;
        double p = std::sqrt(p2 / 6.0);
        if (!(p > 0.0)) {
            l0 = q;
            l1 = q;
            l2 = q;
        } else {
            Sym3 B{
                a00 / p, A.a01 / p, A.a02 / p,
                a11 / p, A.a12 / p,
                a22 / p
            };
            double r = sym_det(B) * 0.5;
            r = clampd(r, -1.0, 1.0);
            constexpr double pi = 3.141592653589793238462643383279502884;
            double phi = std::acos(r) / 3.0;
            l0 = q + 2.0 * p * std::cos(phi);
            l1 = q + 2.0 * p * std::cos(phi + 2.0 * pi / 3.0);
            l2 = q + 2.0 * p * std::cos(phi + 4.0 * pi / 3.0);
        }
    }
    if (l0 < l1) std::swap(l0, l1);
    if (l1 < l2) std::swap(l1, l2);
    if (l0 < l1) std::swap(l0, l1);
}

double PF_LA_zfixed_16x1::sym_cond_2norm(const Sym3& A) {
    double l0, l1, l2;
    sym_eigenvalues(A, l0, l1, l2);
    double maxv = std::max(l0, std::max(l1, l2));
    double minv = std::min(l0, std::min(l1, l2));
    double eps = 1e-18;
    if (!(minv > eps) || !(maxv > eps)) return std::numeric_limits<double>::infinity();
    return maxv / minv;
}

bool PF_LA_zfixed_16x1::solve_linear3(const Sym3& A, const Vec3& b, Vec3& x) {
    double m[3][4] = {
        { A.a00, A.a01, A.a02, b.x },
        { A.a01, A.a11, A.a12, b.y },
        { A.a02, A.a12, A.a22, b.z }
    };

    for (int col = 0; col < 3; ++col) {
        int piv = col;
        double best = std::abs(m[col][col]);
        for (int r = col + 1; r < 3; ++r) {
            double v = std::abs(m[r][col]);
            if (v > best) {
                best = v;
                piv = r;
            }
        }
        if (!(best > 1e-18)) return false;
        if (piv != col) {
            for (int c = col; c < 4; ++c) std::swap(m[col][c], m[piv][c]);
        }
        double invp = 1.0 / m[col][col];
        for (int c = col; c < 4; ++c) m[col][c] *= invp;
        for (int r = 0; r < 3; ++r) {
            if (r == col) continue;
            double f = m[r][col];
            if (f == 0.0) continue;
            for (int c = col; c < 4; ++c) m[r][c] -= f * m[col][c];
        }
    }

    x = Vec3{ m[0][3], m[1][3], m[2][3] };
    return true;
}

PF_LA_zfixed_16x1::Vec3 PF_LA_zfixed_16x1::build_H_row(double x, double y, double z, int sensor_id) const {
    double dx = x - sx_[static_cast<size_t>(sensor_id)];
    double dy = y - sy_[static_cast<size_t>(sensor_id)];
    double z_eff = z + kZ0;
    double Z = kAlpha * z_eff;
    double r2 = dx * dx + dy * dy + Z * Z;
    double r2sq = r2 * r2 + kEps;
    double inv_r2 = 1.0 / (r2 + kEps);
    double h0 = -2.0 * z_eff * dx / r2sq;
    double h1 = -2.0 * z_eff * dy / r2sq;
    double h2 = inv_r2 - (2.0 * kAlpha * kAlpha * z_eff * z_eff) / r2sq;
    return Vec3{ kKprime * h0, kKprime * h1, kKprime * h2 };
}

bool PF_LA_zfixed_16x1::solve_vprime_and_stats(double x, double y, double z, const Mat3& RT, const std::array<double, kUsed>& s, const Vec3& vprime_prev, const Vec3& lam_diag, double ridge, Vec3& vprime, double& J_obs, double& condAobs) const {
    std::array<Vec3, kUsed> hprime_rows;
    Sym3 Aobs = sym_zero();
    Vec3 bobs{ 0.0, 0.0, 0.0 };

    const auto& ids = used_ids();
    for (int i = 0; i < kUsed; ++i) {
        int sid = ids[static_cast<size_t>(i)];
        Vec3 h = build_H_row(x, y, z, sid);
        Vec3 hp = row_mul_mat(h, RT);
        hprime_rows[static_cast<size_t>(i)] = hp;
        Aobs = sym_outer_add(Aobs, hp);
        double si = s[static_cast<size_t>(i)];
        bobs.x += hp.x * si;
        bobs.y += hp.y * si;
        bobs.z += hp.z * si;
    }

    Sym3 Aobs_ridge = sym_add_diag(Aobs, ridge, ridge, ridge);
    condAobs = sym_cond_2norm(Aobs_ridge);

    Sym3 A = sym_add_diag(Aobs, ridge + lam_diag.x, ridge + lam_diag.y, ridge + lam_diag.z);
    Vec3 b{
        bobs.x + lam_diag.x * vprime_prev.x,
        bobs.y + lam_diag.y * vprime_prev.y,
        bobs.z + lam_diag.z * vprime_prev.z
    };

    Vec3 xsol;
    bool ok = solve_linear3(A, b, xsol);
    if (!ok) {
        vprime = vprime_prev;
        J_obs = std::numeric_limits<double>::infinity();
        return false;
    }
    vprime = xsol;

    double e2 = 0.0;
    for (int i = 0; i < kUsed; ++i) {
        const Vec3& hp = hprime_rows[static_cast<size_t>(i)];
        double pred = hp.x * vprime.x + hp.y * vprime.y + hp.z * vprime.z;
        double err = s[static_cast<size_t>(i)] - pred;
        e2 += err * err;
    }
    Vec3 dv = v_sub(vprime, vprime_prev);
    double dv_pen = lam_diag.x * dv.x * dv.x + lam_diag.y * dv.y * dv.y + lam_diag.z * dv.z * dv.z;
    J_obs = e2 + dv_pen;
    return true;
}

bool PF_LA_zfixed_16x1::refine_x_1d(double x0_guess, double y, double z, const std::array<double, kUsed>& s, const Mat3& R, const Mat3& RT, double sigma_eff, const Vec3& lam_diag, const Vec3& v_prev_world, double& x_best, Vec3& v_best_world) const {
    double step = p_.x_step_m;
    if (!(step > 0.0)) return false;

    Vec3 vprime_prev = mat_mul_vec(R, v_prev_world);
    double x_lo = x0_guess - p_.x_span_m;
    double x_hi = x0_guess + p_.x_span_m;
    if (x_hi < x_lo) std::swap(x_lo, x_hi);

    double span = x_hi - x_lo;
    int n_steps = static_cast<int>(std::floor(span / step + 1.0));
    n_steps = clampi(n_steps, 1, 2001);

    bool have_best = false;
    double bestJ = 0.0;
    double bestx = x0_guess;
    Vec3 bestv{ 0.0, 0.0, 0.0 };

    for (int i = 0; i < n_steps; ++i) {
        double xcand = x_lo + step * static_cast<double>(i);
        Vec3 vprime;
        double J_obs = 0.0;
        double cnd = 0.0;
        bool ok = solve_vprime_and_stats(xcand, y, z, RT, s, vprime_prev, lam_diag, p_.ridge, vprime, J_obs, cnd);
        if (!ok) continue;

        Vec3 v_world = mat_mul_vec(RT, vprime);

        double se = sigma_eff;
        if (cnd > p_.cond_thr) {
            double g = std::sqrt(cnd / p_.cond_thr);
            se = sigma_eff * (1.0 + p_.cond_gain * g);
        }

        double vv = v_dot(v_world, v_world);
        double vvyz = v_world.y * v_world.y + v_world.z * v_world.z;
        double J = (J_obs / (se * se)) + kVpen * vv + kVpenYZ * vvyz;

        if (!have_best || J < bestJ) {
            have_best = true;
            bestJ = J;
            bestx = xcand;
            bestv = v_world;
        }
    }

    if (!have_best) return false;
    x_best = bestx;
    v_best_world = bestv;
    return true;
}

void PF_LA_zfixed_16x1::ensure_alloc() {
    int P = std::max(1, p_.particles);
    if (static_cast<int>(pos_.size()) != P) {
        pos_.assign(static_cast<size_t>(P), Vec3{ 0.0, 0.0, p_.z_fixed_m });
        vel_.assign(static_cast<size_t>(P), Vec3{ 0.0, 0.0, 0.0 });
        pos_tmp_.assign(static_cast<size_t>(P), Vec3{ 0.0, 0.0, p_.z_fixed_m });
        vel_tmp_.assign(static_cast<size_t>(P), Vec3{ 0.0, 0.0, 0.0 });
        v_star_.assign(static_cast<size_t>(P), Vec3{ 0.0, 0.0, 0.0 });
        weights_.assign(static_cast<size_t>(P), 1.0 / static_cast<double>(P));
        logw_.assign(static_cast<size_t>(P), 0.0);
        idx_buf_.assign(static_cast<size_t>(P), 0);
    } else {
        if (static_cast<int>(weights_.size()) != P) weights_.assign(static_cast<size_t>(P), 1.0 / static_cast<double>(P));
        if (static_cast<int>(logw_.size()) != P) logw_.assign(static_cast<size_t>(P), 0.0);
        if (static_cast<int>(v_star_.size()) != P) v_star_.assign(static_cast<size_t>(P), Vec3{ 0.0, 0.0, 0.0 });
        if (static_cast<int>(pos_tmp_.size()) != P) pos_tmp_.assign(static_cast<size_t>(P), Vec3{ 0.0, 0.0, p_.z_fixed_m });
        if (static_cast<int>(vel_tmp_.size()) != P) vel_tmp_.assign(static_cast<size_t>(P), Vec3{ 0.0, 0.0, 0.0 });
        if (static_cast<int>(idx_buf_.size()) != P) idx_buf_.assign(static_cast<size_t>(P), 0);
    }
}

void PF_LA_zfixed_16x1::systematic_resample() {
    int P = std::max(1, p_.particles);
    double invP = 1.0 / static_cast<double>(P);
    double u0 = unif01_(rng_) * invP;

    double c = weights_[0];
    int idx = 0;

    for (int i = 0; i < P; ++i) {
        double u = u0 + static_cast<double>(i) * invP;
        while (u > c && idx + 1 < P) {
            ++idx;
            c += weights_[static_cast<size_t>(idx)];
        }
        pos_tmp_[static_cast<size_t>(i)] = pos_[static_cast<size_t>(idx)];
        vel_tmp_[static_cast<size_t>(i)] = vel_[static_cast<size_t>(idx)];
    }

    pos_.swap(pos_tmp_);
    vel_.swap(vel_tmp_);

    for (int i = 0; i < P; ++i) weights_[static_cast<size_t>(i)] = invP;
}

PF_LA_zfixed_16x1::PF_LA_zfixed_16x1()
    : p_{}
    , sx_(default_sensor_x())
    , sy_(default_sensor_y())
    , pos_()
    , vel_()
    , pos_tmp_()
    , vel_tmp_()
    , v_star_()
    , weights_()
    , logw_()
    , idx_buf_()
    , v_prev_{}
    , dvdt_{}
    , have_prev_(false)
    , prev_t_ns_(0)
    , step_count_(0)
    , sref_ema_(0.0)
    , R_(mat_identity())
    , q_dir_{ 1.0, 0.0, 0.0 }
    , rng_(0)
    , norm01_(0.0, 1.0)
    , unif01_(0.0, 1.0) {
    set_params(defaults());
}

const std::string& PF_LA_zfixed_16x1::id() const {
    return kAlgoId;
}

int PF_LA_zfixed_16x1::N() const {
    return kN;
}

int PF_LA_zfixed_16x1::M() const {
    return kM;
}

std::vector<ParamDesc> PF_LA_zfixed_16x1::params() const {
    std::vector<ParamDesc> ps;
    ps.reserve(29);

    ParamDesc d;

    d.key = "particles"; d.label = "Particles"; d.minv = 50.0; d.maxv = 5000.0; d.defv = 900.0; d.step = 10.0; d.decimals = 0; d.scientific = false; ps.push_back(d);
    d.key = "seed"; d.label = "RNG seed"; d.minv = 0.0; d.maxv = 4294967295.0; d.defv = 0.0; d.step = 1.0; d.decimals = 0; d.scientific = false; ps.push_back(d);

    d.key = "z_fixed_m"; d.label = "Z fixed (m)"; d.minv = 0.0; d.maxv = 0.05; d.defv = 0.018; d.step = 0.001; d.decimals = 4; d.scientific = false; ps.push_back(d);

    d.key = "k_dv_s"; d.label = "k_dv (s)"; d.minv = 0.0; d.maxv = 0.2; d.defv = 0.0325; d.step = 0.0005; d.decimals = 4; d.scientific = false; ps.push_back(d);
    d.key = "deriv_tau_s"; d.label = "dV/dt smooth tau (s)"; d.minv = 0.0; d.maxv = 0.2; d.defv = 0.025; d.step = 0.001; d.decimals = 4; d.scientific = false; ps.push_back(d);

    d.key = "pos_noise_m"; d.label = "Process pos noise (m)"; d.minv = 0.0; d.maxv = 0.02; d.defv = 0.0018; d.step = 0.0001; d.decimals = 5; d.scientific = false; ps.push_back(d);
    d.key = "vel_noise_mps"; d.label = "Process vel noise (m/s)"; d.minv = 0.0; d.maxv = 1.0; d.defv = 0.08; d.step = 0.01; d.decimals = 3; d.scientific = false; ps.push_back(d);
    d.key = "jitter_m"; d.label = "Resample jitter (m)"; d.minv = 0.0; d.maxv = 0.02; d.defv = 0.001; d.step = 0.0001; d.decimals = 5; d.scientific = false; ps.push_back(d);

    d.key = "ridge"; d.label = "Ridge"; d.minv = 0.0; d.maxv = 1e-2; d.defv = 1e-6; d.step = 1e-6; d.decimals = 6; d.scientific = true; ps.push_back(d);

    d.key = "lam_vx"; d.label = "Lambda vx"; d.minv = 0.0; d.maxv = 0.2; d.defv = 8e-3; d.step = 1e-3; d.decimals = 5; d.scientific = true; ps.push_back(d);
    d.key = "lam_vy"; d.label = "Lambda vy"; d.minv = 0.0; d.maxv = 0.2; d.defv = 8e-3; d.step = 1e-3; d.decimals = 5; d.scientific = true; ps.push_back(d);
    d.key = "lam_vz"; d.label = "Lambda vz"; d.minv = 0.0; d.maxv = 0.2; d.defv = 3e-2; d.step = 1e-3; d.decimals = 5; d.scientific = true; ps.push_back(d);

    d.key = "vel_mix"; d.label = "Vel mix"; d.minv = 0.0; d.maxv = 1.0; d.defv = 0.02; d.step = 0.01; d.decimals = 3; d.scientific = false; ps.push_back(d);

    d.key = "cond_thr"; d.label = "Cond threshold"; d.minv = 1e2; d.maxv = 1e12; d.defv = 1e6; d.step = 1e4; d.decimals = 0; d.scientific = true; ps.push_back(d);
    d.key = "cond_gain"; d.label = "Cond gain"; d.minv = 0.0; d.maxv = 20.0; d.defv = 2.5; d.step = 0.1; d.decimals = 2; d.scientific = false; ps.push_back(d);

    d.key = "ess_ratio"; d.label = "ESS ratio"; d.minv = 0.05; d.maxv = 0.95; d.defv = 0.30; d.step = 0.01; d.decimals = 2; d.scientific = false; ps.push_back(d);

    d.key = "respawn_ratio"; d.label = "Respawn ratio"; d.minv = 0.0; d.maxv = 0.5; d.defv = 0.05; d.step = 0.01; d.decimals = 2; d.scientific = false; ps.push_back(d);
    d.key = "respawn_info_min"; d.label = "Respawn info min"; d.minv = 0.0; d.maxv = 1.0; d.defv = 0.20; d.step = 0.01; d.decimals = 2; d.scientific = false; ps.push_back(d);

    d.key = "p_info"; d.label = "Info power"; d.minv = 0.2; d.maxv = 6.0; d.defv = 2.0; d.step = 0.1; d.decimals = 2; d.scientific = false; ps.push_back(d);

    d.key = "sigma_gain"; d.label = "Sigma gain"; d.minv = 0.0; d.maxv = 10.0; d.defv = 2.6; d.step = 0.1; d.decimals = 2; d.scientific = false; ps.push_back(d);
    d.key = "lam_gain"; d.label = "Lambda gain"; d.minv = 0.0; d.maxv = 50.0; d.defv = 10.0; d.step = 0.5; d.decimals = 1; d.scientific = false; ps.push_back(d);
    d.key = "w_floor_max"; d.label = "Weight floor max"; d.minv = 0.0; d.maxv = 0.3; d.defv = 0.03; d.step = 0.01; d.decimals = 2; d.scientific = false; ps.push_back(d);

    d.key = "vel_damp_max"; d.label = "Vel damp max"; d.minv = 0.0; d.maxv = 1.0; d.defv = 0.18; d.step = 0.01; d.decimals = 2; d.scientific = false; ps.push_back(d);

    d.key = "x_refine_on"; d.label = "X refine on"; d.minv = 0.0; d.maxv = 1.0; d.defv = 1.0; d.step = 1.0; d.decimals = 0; d.scientific = false; ps.push_back(d);
    d.key = "x_span_m"; d.label = "X refine span (m)"; d.minv = 0.0; d.maxv = 0.1; d.defv = 0.014; d.step = 0.001; d.decimals = 4; d.scientific = false; ps.push_back(d);
    d.key = "x_step_m"; d.label = "X refine step (m)"; d.minv = 0.0001; d.maxv = 0.01; d.defv = 0.0005; d.step = 0.0001; d.decimals = 5; d.scientific = false; ps.push_back(d);
    d.key = "x_refine_gain"; d.label = "X refine gain"; d.minv = 0.0; d.maxv = 2.0; d.defv = 0.35; d.step = 0.01; d.decimals = 2; d.scientific = false; ps.push_back(d);
    d.key = "x_refine_info_min"; d.label = "X refine info min"; d.minv = 0.0; d.maxv = 1.0; d.defv = 0.22; d.step = 0.01; d.decimals = 2; d.scientific = false; ps.push_back(d);
    d.key = "dx_clamp_m"; d.label = "X refine clamp (m)"; d.minv = 0.0; d.maxv = 0.05; d.defv = 0.003; d.step = 0.0005; d.decimals = 4; d.scientific = false; ps.push_back(d);

    return ps;
}

std::vector<double> PF_LA_zfixed_16x1::defaults() const {
    return {
        900.0,
        0.0,
        0.018,
        0.0325,
        0.025,
        0.0018,
        0.08,
        0.001,
        1e-6,
        8e-3,
        8e-3,
        3e-2,
        0.02,
        1e6,
        2.5,
        0.30,
        0.05,
        0.20,
        2.0,
        2.6,
        10.0,
        0.03,
        0.18,
        1.0,
        0.014,
        0.0005,
        0.35,
        0.22,
        0.003
    };
}

void PF_LA_zfixed_16x1::set_params(const std::vector<double>& values) {
    std::vector<double> d = defaults();
    std::vector<double> v = d;
    size_t n = std::min(v.size(), values.size());
    for (size_t i = 0; i < n; ++i) v[i] = values[i];

    int particles = clampi(static_cast<int>(std::llround(v[0])), 50, 5000);
    uint64_t seed = clampu64(static_cast<uint64_t>(std::llround(clampd(v[1], 0.0, 4294967295.0))), 0ull, 4294967295ull);

    double z_fixed_m = clampd(v[2], 0.0, 0.05);
    double k_dv_s = clampd(v[3], 0.0, 0.2);
    double deriv_tau_s = clampd(v[4], 0.0, 0.2);

    double pos_noise_m = clampd(v[5], 0.0, 0.02);
    double vel_noise_mps = clampd(v[6], 0.0, 1.0);
    double jitter_m = clampd(v[7], 0.0, 0.02);

    double ridge = clampd(v[8], 0.0, 1e-2);

    double lam_vx = clampd(v[9], 0.0, 0.2);
    double lam_vy = clampd(v[10], 0.0, 0.2);
    double lam_vz = clampd(v[11], 0.0, 0.2);

    double vel_mix = clampd(v[12], 0.0, 1.0);

    double cond_thr = clampd(v[13], 1e2, 1e12);
    double cond_gain = clampd(v[14], 0.0, 20.0);

    double ess_ratio = clampd(v[15], 0.05, 0.95);

    double respawn_ratio = clampd(v[16], 0.0, 0.5);
    double respawn_info_min = clampd(v[17], 0.0, 1.0);

    double p_info = clampd(v[18], 0.2, 6.0);

    double sigma_gain = clampd(v[19], 0.0, 10.0);
    double lam_gain = clampd(v[20], 0.0, 50.0);
    double w_floor_max = clampd(v[21], 0.0, 0.3);

    double vel_damp_max = clampd(v[22], 0.0, 1.0);

    double x_refine_on = clampd(v[23], 0.0, 1.0);
    double x_span_m = clampd(v[24], 0.0, 0.1);
    double x_step_m = clampd(v[25], 0.0001, 0.01);
    double x_refine_gain = clampd(v[26], 0.0, 2.0);
    double x_refine_info_min = clampd(v[27], 0.0, 1.0);
    double dx_clamp_m = clampd(v[28], 0.0, 0.05);

    p_.particles = particles;
    p_.seed = seed;
    p_.z_fixed_m = z_fixed_m;
    p_.k_dv_s = k_dv_s;
    p_.deriv_tau_s = deriv_tau_s;
    p_.pos_noise_m = pos_noise_m;
    p_.vel_noise_mps = vel_noise_mps;
    p_.jitter_m = jitter_m;
    p_.ridge = ridge;
    p_.lam_vx = lam_vx;
    p_.lam_vy = lam_vy;
    p_.lam_vz = lam_vz;
    p_.vel_mix = vel_mix;
    p_.cond_thr = cond_thr;
    p_.cond_gain = cond_gain;
    p_.ess_ratio = ess_ratio;
    p_.respawn_ratio = respawn_ratio;
    p_.respawn_info_min = respawn_info_min;
    p_.p_info = p_info;
    p_.sigma_gain = sigma_gain;
    p_.lam_gain = lam_gain;
    p_.w_floor_max = w_floor_max;
    p_.vel_damp_max = vel_damp_max;
    p_.x_refine_on = x_refine_on;
    p_.x_span_m = x_span_m;
    p_.x_step_m = x_step_m;
    p_.x_refine_gain = x_refine_gain;
    p_.x_refine_info_min = x_refine_info_min;
    p_.dx_clamp_m = dx_clamp_m;

    reset();
}

void PF_LA_zfixed_16x1::reset() {
    rng_.seed(p_.seed);
    ensure_alloc();

    int P = std::max(1, p_.particles);
    double invP = 1.0 / static_cast<double>(P);

    for (int i = 0; i < P; ++i) {
        pos_[static_cast<size_t>(i)].x = norm01_(rng_) * kInitPosStd;
        pos_[static_cast<size_t>(i)].y = norm01_(rng_) * kInitPosStd;
        pos_[static_cast<size_t>(i)].z = p_.z_fixed_m;

        vel_[static_cast<size_t>(i)].x = norm01_(rng_) * kInitVelStdXY;
        vel_[static_cast<size_t>(i)].y = norm01_(rng_) * kInitVelStdXY;
        vel_[static_cast<size_t>(i)].z = norm01_(rng_) * kInitVelStdZ;

        weights_[static_cast<size_t>(i)] = invP;
        logw_[static_cast<size_t>(i)] = 0.0;
        v_star_[static_cast<size_t>(i)] = Vec3{ 0.0, 0.0, 0.0 };
        idx_buf_[static_cast<size_t>(i)] = i;
    }

    v_prev_.fill(0.0);
    dvdt_.fill(0.0);
    have_prev_ = false;
    prev_t_ns_ = 0;
    step_count_ = 0;

    sref_ema_ = 0.0;
    R_ = mat_identity();
    q_dir_ = Vec3{ 1.0, 0.0, 0.0 };
}

bool PF_LA_zfixed_16x1::push_sample(uint64_t t_ns, const std::vector<float>& sample, Output& out) {
    if (sample.size() != static_cast<size_t>(kN)) return false;

    int P = std::max(1, p_.particles);
    if (static_cast<int>(pos_.size()) != P) ensure_alloc();

    double dt = 0.005;
    if (have_prev_) {
        uint64_t dtns = t_ns >= prev_t_ns_ ? (t_ns - prev_t_ns_) : 0ull;
        dt = static_cast<double>(dtns) * 1e-9;
        if (!(dt > 0.0)) dt = 0.005;
        dt = clampd(dt, 1e-4, 0.1);
    }

    double deriv_alpha = 1.0;
    if (p_.deriv_tau_s > 0.0) deriv_alpha = dt / (p_.deriv_tau_s + dt);

    std::array<double, kN> V;
    for (int i = 0; i < kN; ++i) V[static_cast<size_t>(i)] = static_cast<double>(sample[static_cast<size_t>(i)]);

    if (have_prev_) {
        for (int i = 0; i < kN; ++i) {
            size_t ii = static_cast<size_t>(i);
            double raw = (V[ii] - v_prev_[ii]) / dt;
            dvdt_[ii] = (1.0 - deriv_alpha) * dvdt_[ii] + deriv_alpha * raw;
        }
    } else {
        dvdt_.fill(0.0);
    }

    std::array<double, kN> S;
    for (int i = 0; i < kN; ++i) {
        size_t ii = static_cast<size_t>(i);
        S[ii] = V[ii] + p_.k_dv_s * dvdt_[ii];
    }

    v_prev_ = V;
    prev_t_ns_ = t_ns;
    have_prev_ = true;

    std::array<double, kUsed> s_used;
    const auto& ids = used_ids();
    for (int i = 0; i < kUsed; ++i) {
        int sid = ids[static_cast<size_t>(i)];
        s_used[static_cast<size_t>(i)] = S[static_cast<size_t>(sid)];
    }

    double sn2 = 0.0;
    for (int i = 0; i < kUsed; ++i) {
        double v = s_used[static_cast<size_t>(i)];
        sn2 += v * v;
    }
    double sn = std::sqrt(sn2);

    double sref_alpha = dt / (kSRefTauS + dt);
    if (!(sref_ema_ > 0.0) || step_count_ == 0) sref_ema_ = sn;
    else sref_ema_ = (1.0 - sref_alpha) * sref_ema_ + sref_alpha * sn;

    double S_ref = std::max(sref_ema_, 1e-12);
    double info = std::pow(sn / S_ref, p_.p_info);
    info = clampd(info, kInfoMin, kInfoMax);

    double sigma_base = kSigmaBaseFactor * (S_ref / std::sqrt(static_cast<double>(kUsed)));
    double sigma_eff = sigma_base * (1.0 + p_.sigma_gain * (1.0 / info - 1.0));

    double lam_scale = 1.0 + p_.lam_gain * (1.0 / info - 1.0);
    Vec3 lam_diag{
        p_.lam_vx * lam_scale,
        p_.lam_vy * lam_scale,
        p_.lam_vz * lam_scale
    };

    double w_floor = p_.w_floor_max * (1.0 - info);

    Vec3 pos_c{ 0.0, 0.0, p_.z_fixed_m };
    for (int i = 0; i < P; ++i) {
        double w = weights_[static_cast<size_t>(i)];
        pos_c.x += w * pos_[static_cast<size_t>(i)].x;
        pos_c.y += w * pos_[static_cast<size_t>(i)].y;
    }

    Vec3 a{ 0.0, 0.0, 0.0 };
    for (int i = 0; i < kUsed; ++i) {
        int sid = ids[static_cast<size_t>(i)];
        Vec3 h = build_H_row(pos_c.x, pos_c.y, p_.z_fixed_m, sid);
        double si = s_used[static_cast<size_t>(i)];
        a.x += h.x * si;
        a.y += h.y * si;
        a.z += h.z * si;
    }
    a = v_normalize(a, 1e-12);

    double beta = kBetaDirGain * std::pow(info, kBetaDirPow);
    beta = clampd(beta, 0.0, 1.0);
    q_dir_ = v_add(v_scale(q_dir_, 1.0 - beta), v_scale(a, beta));
    Vec3 u_hat = v_normalize(q_dir_, 1e-12);
    if (v_norm(u_hat) < 1e-12) u_hat = Vec3{ 1.0, 0.0, 0.0 };

    Vec3 target_dir = v_normalize(Vec3{ 1.0, 1.0, 1.0 }, 1e-12);
    Mat3 R_new = rot_a_to_b(u_hat, target_dir);

    double gamma_eff = kRotGamma * std::pow(info, kRotGammaPow);
    gamma_eff = clampd(gamma_eff, kRotGammaMin, kRotGammaMax);
    R_ = smooth_rot(R_, R_new, gamma_eff);
    Mat3 RT = mat_transpose(R_);

    double damp = p_.vel_damp_max * std::pow(1.0 - info, kVelDampPow);
    damp = clampd(damp, 0.0, 1.0);
    if (damp > 0.0) {
        double s = 1.0 - damp;
        for (int i = 0; i < P; ++i) vel_[static_cast<size_t>(i)] = v_scale(vel_[static_cast<size_t>(i)], s);
    }

    if (step_count_ > 0) {
        double vel_scale = std::max(kVelScaleMin, std::pow(info, kVelNoiseScalePow));
        double pos_noise = p_.pos_noise_m * vel_scale;
        double vel_noise = p_.vel_noise_mps * vel_scale;

        for (int i = 0; i < P; ++i) {
            Vec3& pp = pos_[static_cast<size_t>(i)];
            Vec3& vv = vel_[static_cast<size_t>(i)];
            pp.x += vv.x * dt + norm01_(rng_) * pos_noise;
            pp.y += vv.y * dt + norm01_(rng_) * pos_noise;
            pp.z = p_.z_fixed_m;

            vv.x += norm01_(rng_) * vel_noise;
            vv.y += norm01_(rng_) * vel_noise;
            vv.z += norm01_(rng_) * (0.5 * vel_noise);

            pp.z = p_.z_fixed_m;
        }
    } else {
        for (int i = 0; i < P; ++i) pos_[static_cast<size_t>(i)].z = p_.z_fixed_m;
    }

    for (int i = 0; i < P; ++i) {
        Vec3 vprime_prev = mat_mul_vec(R_, vel_[static_cast<size_t>(i)]);
        Vec3 vprime;
        double J_obs = 0.0;
        double cnd = 0.0;

        solve_vprime_and_stats(
            pos_[static_cast<size_t>(i)].x,
            pos_[static_cast<size_t>(i)].y,
            p_.z_fixed_m,
            RT,
            s_used,
            vprime_prev,
            lam_diag,
            p_.ridge,
            vprime,
            J_obs,
            cnd
        );

        Vec3 v_world = mat_mul_vec(RT, vprime);
        v_star_[static_cast<size_t>(i)] = v_world;

        double se = sigma_eff;
        if (cnd > p_.cond_thr) {
            double g = std::sqrt(cnd / p_.cond_thr);
            se = sigma_eff * (1.0 + p_.cond_gain * g);
        }

        double vv = v_dot(v_world, v_world);
        double vvyz = v_world.y * v_world.y + v_world.z * v_world.z;
        double J = (J_obs / (se * se)) + kVpen * vv + kVpenYZ * vvyz;
        logw_[static_cast<size_t>(i)] = -0.5 * J;
    }

    double maxlog = logw_[0];
    for (int i = 1; i < P; ++i) {
        double v = logw_[static_cast<size_t>(i)];
        if (v > maxlog) maxlog = v;
    }

    double sumw = 0.0;
    for (int i = 0; i < P; ++i) {
        double w = std::exp(logw_[static_cast<size_t>(i)] - maxlog);
        weights_[static_cast<size_t>(i)] = w;
        sumw += w;
    }

    if (!(sumw > 0.0)) {
        double invP = 1.0 / static_cast<double>(P);
        for (int i = 0; i < P; ++i) weights_[static_cast<size_t>(i)] = invP;
    } else {
        double invsum = 1.0 / sumw;
        for (int i = 0; i < P; ++i) weights_[static_cast<size_t>(i)] *= invsum;
    }

    if (w_floor > 0.0) {
        double invP = 1.0 / static_cast<double>(P);
        double s0 = 1.0 - w_floor;
        double s1 = w_floor * invP;
        double ssum = 0.0;
        for (int i = 0; i < P; ++i) {
            double w = s0 * weights_[static_cast<size_t>(i)] + s1;
            weights_[static_cast<size_t>(i)] = w;
            ssum += w;
        }
        if (ssum > 0.0) {
            double inv = 1.0 / ssum;
            for (int i = 0; i < P; ++i) weights_[static_cast<size_t>(i)] *= inv;
        } else {
            for (int i = 0; i < P; ++i) weights_[static_cast<size_t>(i)] = invP;
        }
    }

    double mix = p_.vel_mix;
    mix = clampd(mix, 0.0, 1.0);
    double mix0 = 1.0 - mix;

    for (int i = 0; i < P; ++i) {
        Vec3& vv = vel_[static_cast<size_t>(i)];
        const Vec3& vs = v_star_[static_cast<size_t>(i)];
        vv.x = mix0 * vv.x + mix * vs.x;
        vv.y = mix0 * vv.y + mix * vs.y;
        vv.z = mix0 * vv.z + mix * vs.z;
    }

    Vec3 est_pos{ 0.0, 0.0, p_.z_fixed_m };
    Vec3 est_vel{ 0.0, 0.0, 0.0 };

    double sumw2 = 0.0;
    for (int i = 0; i < P; ++i) {
        double w = weights_[static_cast<size_t>(i)];
        const Vec3& pp = pos_[static_cast<size_t>(i)];
        const Vec3& vv = vel_[static_cast<size_t>(i)];
        est_pos.x += w * pp.x;
        est_pos.y += w * pp.y;
        est_vel.x += w * vv.x;
        est_vel.y += w * vv.y;
        est_vel.z += w * vv.z;
        sumw2 += w * w;
    }

    double ESS = 1.0 / (sumw2 + 1e-18);

    Sym3 Aobs_est = sym_zero();
    for (int i = 0; i < kUsed; ++i) {
        int sid = ids[static_cast<size_t>(i)];
        Vec3 h = build_H_row(est_pos.x, est_pos.y, p_.z_fixed_m, sid);
        Aobs_est = sym_outer_add(Aobs_est, h);
    }
    Sym3 Aobs_est_r = sym_add_diag(Aobs_est, p_.ridge, p_.ridge, p_.ridge);
    double cond_est = sym_cond_2norm(Aobs_est_r);

    if (p_.x_refine_on > 0.5 && info >= p_.x_refine_info_min) {
        double x_best = est_pos.x;
        Vec3 v_best{ est_vel.x, est_vel.y, est_vel.z };
        bool ok = refine_x_1d(est_pos.x, est_pos.y, p_.z_fixed_m, s_used, R_, RT, sigma_eff, lam_diag, est_vel, x_best, v_best);
        if (ok) {
            double dx = clampd(x_best - est_pos.x, -p_.dx_clamp_m, p_.dx_clamp_m);
            double g = p_.x_refine_gain * std::pow(info, kXRefineGainPow);
            if (g > 0.0) {
                for (int i = 0; i < P; ++i) pos_[static_cast<size_t>(i)].x += g * dx;
                est_pos.x += dx;
                est_vel.x = v_best.x;
            }
        }
    }

    if (ESS < static_cast<double>(P) * p_.ess_ratio) {
        systematic_resample();

        if (p_.jitter_m > 0.0) {
            for (int i = 0; i < P; ++i) {
                pos_[static_cast<size_t>(i)].x += norm01_(rng_) * p_.jitter_m;
                pos_[static_cast<size_t>(i)].y += norm01_(rng_) * p_.jitter_m;
                pos_[static_cast<size_t>(i)].z = p_.z_fixed_m;
            }
        }

        int Rn = 0;
        if (info >= p_.respawn_info_min) {
            Rn = static_cast<int>(std::llround(p_.respawn_ratio * static_cast<double>(P)));
            Rn = clampi(Rn, 0, P);
        }

        if (Rn > 0) {
            for (int i = 0; i < P; ++i) idx_buf_[static_cast<size_t>(i)] = i;
            for (int i = 0; i < Rn; ++i) {
                std::uniform_int_distribution<int> uid(i, P - 1);
                int j = uid(rng_);
                std::swap(idx_buf_[static_cast<size_t>(i)], idx_buf_[static_cast<size_t>(j)]);
            }
            for (int i = 0; i < Rn; ++i) {
                int k = idx_buf_[static_cast<size_t>(i)];
                pos_[static_cast<size_t>(k)].x = est_pos.x + norm01_(rng_) * kRespawnPosStd;
                pos_[static_cast<size_t>(k)].y = est_pos.y + norm01_(rng_) * kRespawnPosStd;
                pos_[static_cast<size_t>(k)].z = p_.z_fixed_m;

                vel_[static_cast<size_t>(k)].x = est_vel.x + norm01_(rng_) * kRespawnVelStdXY;
                vel_[static_cast<size_t>(k)].y = est_vel.y + norm01_(rng_) * kRespawnVelStdXY;
                vel_[static_cast<size_t>(k)].z = est_vel.z + norm01_(rng_) * kRespawnVelStdZ;
            }
        }
    }

    step_count_ += 1;

    out.x = est_pos.x;
    out.y = est_pos.y;
    set_member_z(out, p_.z_fixed_m, 0);
    set_member_vx(out, est_vel.x, 0);
    set_member_vy(out, est_vel.y, 0);
    set_member_vz(out, est_vel.z, 0);

    double ess_frac = clampd(ESS / static_cast<double>(P), 0.0, 1.0);
    double cond_pen = 1.0;
    if (cond_est > p_.cond_thr && std::isfinite(cond_est) && std::isfinite(p_.cond_thr) && p_.cond_thr > 0.0) {
        cond_pen = 1.0 / (1.0 + std::log10(cond_est / p_.cond_thr));
        cond_pen = clampd(cond_pen, 0.0, 1.0);
    }

    double confidence = info * std::sqrt(ess_frac) * cond_pen;
    confidence = clampd(confidence, 0.0, 1.0);

    out.confidence = confidence;
    out.valid = (confidence >= 0.05) && (info >= 0.05);
    out.quiet = (info < 0.05);

    return true;
}

HUB_PT_REGISTER_ALGORITHM(PF_LA_zfixed_16x1)

}