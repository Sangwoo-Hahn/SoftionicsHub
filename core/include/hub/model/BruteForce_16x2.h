#ifndef HUB_MODEL_BRUTEFORCE_16X2_H
#define HUB_MODEL_BRUTEFORCE_16X2_H

#include <array>
#include <vector>
#include <mutex>
#include <cstddef>

namespace hub {

struct Vec3d { double x; double y; double z; };

struct BruteForce_16x2Output {
    bool has_pose = false;
    bool quiet = false;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double q1 = 0.0;
    double q2 = 0.0;
    double err = 0.0;
};

class BruteForce_16x2Solver {
public:
    static constexpr int NSENS = 16;

    BruteForce_16x2Solver();
    void reset();

    // runtime params
    void set_params(double rc_r, double rc_c, double ema_alpha, double quiet_err_thresh);
    void get_params(double& rc_r, double& rc_c, double& ema_alpha, double& quiet_err_thresh) const;

    // grid params (rebuild)
    void set_grid(double xmin, double xmax,
                  double ymin, double ymax,
                  double zmin, double zmax,
                  double step);

    BruteForce_16x2Output update(const std::vector<float>& v);

    static std::array<Vec3d, NSENS> sensor_positions();

private:
    struct GridPoint { double x; double y; double z; };

    static void ensure_sensors();
    static void build_sensors();
    static double dist3(const Vec3d& a, const Vec3d& b);

    void rebuild_grid();
    int solve_static_idx(const double V[NSENS], Vec3d& out_r, double& out_q, double& out_err);
    int solve_dynamic_idx(const double V1[NSENS], const double V2[NSENS], int idx_r1,
                          Vec3d& out_r2, double& out_q1k, double& out_q2k, double& out_err);

    Vec3d ema_cascade_update(const Vec3d& x);

private:
    static std::once_flag sensors_once_;
    static std::array<Vec3d, NSENS> sensors_;

    std::vector<GridPoint> grid_;
    std::vector<std::array<double, NSENS>> invR_;

    double xmin_ = -0.06, xmax_ = 0.06;
    double ymin_ = -0.06, ymax_ = 0.06;
    double zmin_ =  0.01, zmax_ = 0.10;
    double step_ =  0.01;
    bool grid_built_ = false;

    double RC_R_ = 1e8;
    double RC_C_ = 5e-10;

    double ema_alpha_ = 0.2;
    double quiet_err_thresh_ = 0.3;

    double prevV_[NSENS]{};
    bool hasPrevV_ = false;

    int prevGridIdx_ = -1;
    bool hasPrevR_ = false;

    bool emaInit_[2] = {false, false};
    Vec3d emaState_[2] = {{0,0,0},{0,0,0}};

    bool hasLastEma_ = false;
    Vec3d lastEma_{0,0,0};
};

}

#endif
