#pragma once
#include <cstddef>
#include <vector>

namespace hub {

class MAFilter {
public:
    void reset();
    void configure(size_t n_ch, size_t win_len);
    void process_inplace(std::vector<float>& x);

    size_t win_len() const { return win_len_; }
    bool ready() const { return ready_; }

private:
    bool ready_ = false;
    size_t n_ch_ = 0;
    size_t win_len_ = 1;
    size_t idx_ = 0;
    std::vector<float> sum_;
    std::vector<float> ring_;
};

}
