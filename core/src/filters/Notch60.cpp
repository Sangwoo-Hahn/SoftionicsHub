#define _USE_MATH_DEFINES
#include <cmath>
#include "hub/filters/Notch60.h"

namespace hub {

void NotchBiquad::reset() {
    ready_ = false;
    n_ch_ = 0;
    fs_ = 200.0;
    f0_ = 60.0;
    q_ = 30.0;
    b0_=1; b1_=0; b2_=0; a1_=0; a2_=0;
    z1_.clear();
    z2_.clear();
}

void NotchBiquad::configure(size_t n_ch, double fs_hz, double f0_hz, double q) {
    n_ch_ = n_ch;
    fs_ = fs_hz;
    f0_ = f0_hz;
    q_ = q;
    z1_.assign(n_ch_, 0.0);
    z2_.assign(n_ch_, 0.0);
    recompute();
    ready_ = true;
}

void NotchBiquad::set_params(double fs_hz, double f0_hz, double q) {
    fs_ = fs_hz;
    f0_ = f0_hz;
    q_ = q;
    recompute();
}

void NotchBiquad::recompute() {
    if (fs_ <= 0) fs_ = 200.0;
    if (f0_ <= 0) f0_ = 60.0;
    if (q_ <= 0) q_ = 30.0;

    double w0 = 2.0 * M_PI * (f0_ / fs_);
    double cosw0 = std::cos(w0);
    double sinw0 = std::sin(w0);
    double alpha = sinw0 / (2.0 * q_);

    double b0 = 1.0;
    double b1 = -2.0 * cosw0;
    double b2 = 1.0;
    double a0 = 1.0 + alpha;
    double a1 = -2.0 * cosw0;
    double a2 = 1.0 - alpha;

    b0_ = b0 / a0;
    b1_ = b1 / a0;
    b2_ = b2 / a0;
    a1_ = a1 / a0;
    a2_ = a2 / a0;
}

void NotchBiquad::process_inplace(std::vector<float>& x) {
    if (!ready_) return;
    if (x.size() != n_ch_) return;

    for (size_t i = 0; i < n_ch_; ++i) {
        double in = static_cast<double>(x[i]);
        double out = b0_ * in + z1_[i];
        z1_[i] = b1_ * in - a1_ * out + z2_[i];
        z2_[i] = b2_ * in - a2_ * out;
        x[i] = static_cast<float>(out);
    }
}

}
