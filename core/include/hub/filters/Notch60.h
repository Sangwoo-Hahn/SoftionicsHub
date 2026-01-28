#pragma once
#include <cstddef>
#include <vector>

namespace hub {

class NotchBiquad {
public:
    void reset();
    void configure(size_t n_ch, double fs_hz, double f0_hz, double q);
    void set_params(double fs_hz, double f0_hz, double q);
    void process_inplace(std::vector<float>& x);

    bool ready() const { return ready_; }

private:
    void recompute();

    bool ready_ = false;
    size_t n_ch_ = 0;

    double fs_ = 200.0;
    double f0_ = 60.0;
    double q_ = 30.0;

    double b0_=1, b1_=0, b2_=0, a1_=0, a2_=0;

    std::vector<double> z1_;
    std::vector<double> z2_;
};

}
