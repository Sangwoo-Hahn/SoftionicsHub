#pragma once
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace hub {

class LinearModel {
public:
    void reset();
    void configure(size_t n_ch);
    void set_bias(float b);
    void set_weights(const std::vector<float>& w);
    float eval(const std::vector<float>& x) const;

    bool ready() const { return ready_; }
    const std::vector<float>& weights() const { return w_; }
    float bias() const { return b_; }

private:
    bool ready_ = false;
    size_t n_ch_ = 0;
    std::vector<float> w_;
    float b_ = 0.0f;
};

std::optional<std::vector<float>> load_weights_csv_1line(const std::string& path);

}
