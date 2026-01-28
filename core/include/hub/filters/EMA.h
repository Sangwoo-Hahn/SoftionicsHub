#pragma once
#include <cstddef>
#include <vector>

namespace hub {

class EMAFilter {
public:
    void reset();
    void configure(size_t n_ch, float alpha);
    void set_alpha(float alpha);
    void process_inplace(std::vector<float>& x);

    float alpha() const { return alpha_; }
    bool ready() const { return ready_; }

private:
    bool ready_ = false;
    size_t n_ch_ = 0;
    float alpha_ = 0.2f;
    std::vector<float> y_;
};

}
