#include "hub/filters/MA.h"
#include <algorithm>

namespace hub {

void MAFilter::reset() {
    ready_ = false;
    n_ch_ = 0;
    win_len_ = 1;
    idx_ = 0;
    sum_.clear();
    ring_.clear();
}

void MAFilter::configure(size_t n_ch, size_t win_len) {
    if (win_len < 1) win_len = 1;
    n_ch_ = n_ch;
    win_len_ = win_len;
    idx_ = 0;
    sum_.assign(n_ch_, 0.0f);
    ring_.assign(n_ch_ * win_len_, 0.0f);
    ready_ = true;
}

void MAFilter::process_inplace(std::vector<float>& x) {
    if (!ready_) return;
    if (x.size() != n_ch_) return;

    float inv = 1.0f / static_cast<float>(win_len_);
    size_t base = idx_ * n_ch_;

    for (size_t i = 0; i < n_ch_; ++i) {
        float oldv = ring_[base + i];
        float newv = x[i];
        sum_[i] += (newv - oldv);
        ring_[base + i] = newv;
        x[i] = sum_[i] * inv;
    }

    idx_++;
    if (idx_ >= win_len_) idx_ = 0;
}

}
