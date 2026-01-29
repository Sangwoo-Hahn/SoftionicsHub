#ifndef HUB_PIPELINE_H
#define HUB_PIPELINE_H

#include <cstdint>
#include <vector>
#include <cmath>
#include <algorithm>
#include "hub/filters/Bias.h"

namespace hub {

struct PipelineConfig {
    bool enable_ma = false;
    size_t ma_win = 5;

    bool enable_ema = false;
    float ema_alpha = 0.2f;

    bool enable_notch = false;
    double fs_hz = 200.0;
    double notch_f0 = 60.0;
    double notch_q = 30.0;

    bool enable_bias = false;
};

struct Frame {
    uint64_t t_ns = 0;
    std::vector<float> x;
};

struct PipelineOut {
    Frame frame;
};

class Pipeline {
public:
    void reset();
    void set_config(const PipelineConfig& cfg);

    void ensure_initialized(size_t n_ch);

    PipelineOut process(uint64_t t_ns, const std::vector<float>& in);

    void begin_bias_capture(size_t frames);

    bool bias_has() const { return bias_.has_bias(); }
    bool bias_capturing() const { return bias_.capturing(); }
    const std::vector<float>& bias_vec() const { return bias_.bias(); }

private:
    void ensure_ma();
    void ensure_ema();
    void ensure_notch();
    void update_notch_coeff();

private:
    PipelineConfig cfg_{};
    size_t n_ch_ = 0;

    // MA ring
    bool ma_inited_ = false;
    size_t ma_pos_ = 0;
    std::vector<float> ma_ring_;     // size = n_ch * ma_win
    std::vector<double> ma_sum_;     // size = n_ch

    // EMA
    bool ema_inited_ = false;
    std::vector<float> ema_state_;   // size = n_ch

    // Notch biquad
    bool notch_inited_ = false;
    double b0_=1,b1_=0,b2_=0,a1_=0,a2_=0;
    std::vector<double> nx1_, nx2_, ny1_, ny2_; // per channel

    // Bias
    BiasCorrector bias_;
};

}

#endif
