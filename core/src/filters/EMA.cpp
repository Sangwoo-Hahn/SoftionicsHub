#include "hub/filters/EMA.h"
#include <algorithm>

namespace hub {

void EMAFilter::reset() {
    ready_ = false;
    n_ch_ = 0;
    alpha_ = 0.2f;
    y_.clear();
}

void EMAFilter::configure(size_t n_ch, float alpha) {
    n_ch_ = n_ch;
    set_alpha(alpha);
    y_.assign(n_ch_, 0.0f);
    ready_ = true;
}

void EMAFilter::set_alpha(float alpha) {
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    alpha_ = alpha;
}

void EMAFilter::process_inplace(std::vector<float>& x) {
    if (!ready_) return;
    if (x.size() != n_ch_) return;

    float a = alpha_;
    float b = 1.0f - a;

    for (size_t i = 0; i < n_ch_; ++i) {
        y_[i] = a * x[i] + b * y_[i];
        x[i] = y_[i];
    }
}

}
