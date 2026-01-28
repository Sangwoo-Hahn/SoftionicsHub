#pragma once
#include <cstddef>
#include <vector>

namespace hub {

class BiasCorrector {
public:
    void reset();
    void configure(size_t n_ch);

    void begin_capture(size_t frames);
    bool capturing() const { return capturing_; }
    bool has_bias() const { return has_bias_; }

    void clear_bias();
    void set_bias(const std::vector<float>& bias);

    void update_capture(const std::vector<float>& x);
    void apply_inplace(std::vector<float>& x) const;

    const std::vector<float>& bias() const { return bias_; }
    size_t n_ch() const { return n_ch_; }

private:
    size_t n_ch_ = 0;

    bool capturing_ = false;
    bool has_bias_ = false;

    size_t cap_target_ = 0;
    size_t cap_count_ = 0;

    std::vector<double> acc_;
    std::vector<float> bias_;
};

}
