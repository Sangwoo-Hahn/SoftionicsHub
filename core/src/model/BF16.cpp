#include "hub/model/BF16.h"
#include <cmath>
#include <algorithm>

namespace hub {

std::once_flag BF16Solver::build_once_;
std::array<Vec3d, BF16Solver::NSENS> BF16Solver::sensors_{};
std::vector<BF16Solver::GridPoint> BF16Solver::grid_{};
std::vector<std::array<double, BF16Solver::NSENS>> BF16Solver::invR_{};

BF16Solver::BF16Solver() {
    reset();
    ensure_built();
}

void BF16Solver::reset() {
    hasPrevV_ = false;
    for (int i = 0; i < NSENS; ++i) prevV_[i] = 0.0;
    prevGridIdx_ = -1;
    hasPrevR_ = false;

    emaInit_[0] = emaInit_[1] = false;
    emaState_[0] = {0,0,0};
    emaState_[1] = {0,0,0};

    hasLastEma_ = false;
    lastEma_ = {0,0,0};
}

std::array<Vec3d, BF16Solver::NSENS> BF16Solver::sensor_positions() {
    ensure_built();
    return sensors_;
}

void BF16Solver::ensure_built() {
    std::call_once(build_once_, []() {
        build_sensors();
        build_grid();
    });
}

void BF16Solver::build_sensors() {
    double d = 19.1e-3;
    sensors_[0]  = { -1.5*d, -1.5*d, 0.0 };
    sensors_[1]  = {  0.5*d, -1.5*d, 0.0 };
    sensors_[2]  = {  1.5*d, -1.5*d, 0.0 };
    sensors_[3]  = {  0.5*d, -0.5*d, 0.0 };
    sensors_[4]  = {  1.5*d, -0.5*d, 0.0 };
    sensors_[5]  = {  0.5*d,  0.5*d, 0.0 };
    sensors_[6]  = {  1.5*d,  0.5*d, 0.0 };
    sensors_[7]  = {  0.5*d,  1.5*d, 0.0 };
    sensors_[8]  = {  1.5*d,  1.5*d, 0.0 };
    sensors_[9]  = { -0.5*d,  1.5*d, 0.0 };
    sensors_[10] = { -1.5*d,  1.5*d, 0.0 };
    sensors_[11] = { -0.5*d,  0.5*d, 0.0 };
    sensors_[12] = { -1.5*d,  0.5*d, 0.0 };
    sensors_[13] = { -0.5*d, -0.5*d, 0.0 };
    sensors_[14] = { -1.5*d, -0.5*d, 0.0 };
    sensors_[15] = { -0.5*d, -1.5*d, 0.0 };
}

double BF16Solver::dist3(const Vec3d& a, const Vec3d& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void BF16Solver::build_grid() {
    double xmin = -0.06, xmax = 0.06;
    double ymin = -0.06, ymax = 0.06;
    double zmin =  0.01, zmax = 0.10;
    double step = 0.01;

    for (double x = xmin; x <= xmax + 1e-12; x += step) {
        for (double y = ymin; y <= ymax + 1e-12; y += step) {
            for (double z = zmin; z <= zmax + 1e-12; z += step) {
                grid_.push_back({x,y,z});

                std::array<double, NSENS> inv{};
                Vec3d r{x,y,z};
                for (int j = 0; j < NSENS; ++j) {
                    double d = dist3(r, sensors_[j]);
                    if (d < 1e-9) d = 1e-9;
                    inv[j] = 1.0 / d;
                }
                invR_.push_back(inv);
            }
        }
    }
}

int BF16Solver::solve_static_idx(const double V[NSENS], Vec3d& out_r, double& out_q, double& out_err) {
    ensure_built();

    double best_err = 1e300;
    int best_idx = -1;
    double best_q = 0.0;

    int NG = (int)grid_.size();
    for (int gi = 0; gi < NG; ++gi) {
        const auto& inv = invR_[gi];

        double num = 0.0, den = 0.0;
        for (int j = 0; j < NSENS; ++j) {
            num += V[j] * inv[j];
            den += inv[j] * inv[j];
        }
        if (den < 1e-18) continue;

        double q = num / den;

        double err = 0.0;
        for (int j = 0; j < NSENS; ++j) {
            double Vmodel = q * inv[j];
            double diff = V[j] - Vmodel;
            err += diff * diff;
        }

        if (err < best_err) {
            best_err = err;
            best_idx = gi;
            best_q = q;
        }
    }

    if (best_idx >= 0) out_r = {grid_[best_idx].x, grid_[best_idx].y, grid_[best_idx].z};
    else out_r = {0,0,0};

    out_q = best_q;
    out_err = best_err;
    return best_idx;
}

int BF16Solver::solve_dynamic_idx(const double V1[NSENS], const double V2[NSENS], int idx_r1,
                                 Vec3d& out_r2, double& out_q1k, double& out_q2k, double& out_err) {
    ensure_built();

    if (idx_r1 < 0 || idx_r1 >= (int)grid_.size()) {
        out_r2 = {0,0,0};
        out_q1k = out_q2k = 0.0;
        out_err = 1e300;
        return -1;
    }

    const auto& inv1 = invR_[idx_r1];

    double lhs[NSENS];
    for (int j = 0; j < NSENS; ++j) {
        lhs[j] = (V1[j] + V2[j]) / (2.0 * RC_R_ * RC_C_) + (V2[j] - V1[j]);
    }

    double best_err = 1e300;
    int best_idx = -1;
    double best_q1k = 0.0;
    double best_q2k = 0.0;

    int NG = (int)grid_.size();
    for (int gi = 0; gi < NG; ++gi) {
        const auto& inv2 = invR_[gi];

        double A11 = 0.0, A22 = 0.0, A12 = 0.0;
        double b1 = 0.0, b2 = 0.0;

        for (int j = 0; j < NSENS; ++j) {
            double phi1 = -inv1[j];
            double phi2 =  inv2[j];
            double y = lhs[j];

            A11 += phi1 * phi1;
            A22 += phi2 * phi2;
            A12 += phi1 * phi2;

            b1  += phi1 * y;
            b2  += phi2 * y;
        }

        double det = A11 * A22 - A12 * A12;
        if (std::fabs(det) < 1e-18) continue;

        double q1k = ( A22 * b1 - A12 * b2) / det;
        double q2k = (-A12 * b1 + A11 * b2) / det;

        double err = 0.0;
        for (int j = 0; j < NSENS; ++j) {
            double phi1 = -inv1[j];
            double phi2 =  inv2[j];
            double y = lhs[j];
            double yhat = phi1 * q1k + phi2 * q2k;
            double diff = y - yhat;
            err += diff * diff;
        }

        if (err < best_err) {
            best_err = err;
            best_idx = gi;
            best_q1k = q1k;
            best_q2k = q2k;
        }
    }

    if (best_idx >= 0) out_r2 = {grid_[best_idx].x, grid_[best_idx].y, grid_[best_idx].z};
    else out_r2 = {0,0,0};

    out_q1k = best_q1k;
    out_q2k = best_q2k;
    out_err = best_err;
    return best_idx;
}

Vec3d BF16Solver::ema_cascade_update(const Vec3d& x) {
    Vec3d y_in = x;
    for (int s = 0; s < 2; ++s) {
        if (!emaInit_[s]) {
            emaState_[s] = y_in;
            emaInit_[s] = true;
        } else {
            emaState_[s].x = ema_alpha_ * y_in.x + (1.0 - ema_alpha_) * emaState_[s].x;
            emaState_[s].y = ema_alpha_ * y_in.y + (1.0 - ema_alpha_) * emaState_[s].y;
            emaState_[s].z = ema_alpha_ * y_in.z + (1.0 - ema_alpha_) * emaState_[s].z;
        }
        y_in = emaState_[s];
    }
    return emaState_[1];
}

BF16Output BF16Solver::update(const std::vector<float>& v) {
    BF16Output out;
    if (v.size() != (size_t)NSENS) return out;

    double Vcur[NSENS];
    for (int j = 0; j < NSENS; ++j) Vcur[j] = (double)v[j];

    if (!hasPrevV_) {
        for (int j = 0; j < NSENS; ++j) prevV_[j] = Vcur[j];
        hasPrevV_ = true;
        prevGridIdx_ = -1;
        hasPrevR_ = false;
        out.has_pose = hasLastEma_;
        if (hasLastEma_) { out.x = lastEma_.x; out.y = lastEma_.y; out.z = lastEma_.z; }
        return out;
    }

    double V1[NSENS], V2[NSENS];
    for (int j = 0; j < NSENS; ++j) { V1[j] = prevV_[j]; V2[j] = Vcur[j]; }

    if (!hasPrevR_) {
        Vec3d r1;
        double q_static = 0.0, err_static = 0.0;
        int idx1 = solve_static_idx(V1, r1, q_static, err_static);
        prevGridIdx_ = idx1;
        hasPrevR_ = (idx1 >= 0);
    }

    bool have_r2 = false;
    Vec3d r2_raw{0,0,0};
    double q1k = 0.0, q2k = 0.0, err_dyn = 1e300;

    if (hasPrevR_ && prevGridIdx_ >= 0) {
        int idx2 = solve_dynamic_idx(V1, V2, prevGridIdx_, r2_raw, q1k, q2k, err_dyn);
        if (idx2 >= 0) {
            have_r2 = true;
            prevGridIdx_ = idx2;
            hasPrevR_ = true;
        } else {
            hasPrevR_ = false;
            prevGridIdx_ = -1;
        }
    }

    bool quiet = (have_r2 && err_dyn <= quiet_err_thresh_);

    if (quiet) {
        hasPrevR_ = false;
        prevGridIdx_ = -1;
        out.quiet = true;
        out.q1 = q1k;
        out.q2 = q2k;
        out.err = err_dyn;
        if (hasLastEma_) {
            out.has_pose = true;
            out.x = lastEma_.x;
            out.y = lastEma_.y;
            out.z = lastEma_.z;
        }
    } else {
        if (have_r2) {
            Vec3d r2_ema = ema_cascade_update(r2_raw);
            lastEma_ = r2_ema;
            hasLastEma_ = true;

            out.has_pose = true;
            out.quiet = false;
            out.x = r2_ema.x;
            out.y = r2_ema.y;
            out.z = r2_ema.z;
            out.q1 = q1k;
            out.q2 = q2k;
            out.err = err_dyn;
        } else {
            out.quiet = false;
            if (hasLastEma_) {
                out.has_pose = true;
                out.x = lastEma_.x;
                out.y = lastEma_.y;
                out.z = lastEma_.z;
            }
        }
    }

    for (int j = 0; j < NSENS; ++j) prevV_[j] = Vcur[j];
    hasPrevV_ = true;

    return out;
}

}
