#ifndef HUB_PIPELINE_H
#define HUB_PIPELINE_H

#include <cstddef>
#include <cstdint>
#include <vector>
#include <cmath>
#include <algorithm>

#include "hub/filters/Bias.h"

namespace hub {

struct PipelineConfig {
    // Moving Average
    bool enable_ma = false;
    size_t ma_win = 5;
    size_t ma_order = 1;

    // Exponential Moving Average
    bool enable_ema = false;
    float ema_alpha = 0.2f;
    size_t ema_order = 1;

    // Notch (biquad)
    bool enable_notch = false;
    double fs_hz = 200.0;
    double notch_f0 = 60.0;
    double notch_q = 30.0;
    size_t notch_order = 1;

    // V/RC + dV/dt
    bool enable_vrc = false;
    double vrc_rc = 0.05;     // RC product (seconds), default 0.05
    size_t vrc_n = 5;         // points for linear slope estimate, default 5
    size_t vrc_order = 1;

    // Bias correction
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
    void ensure_vrc();

    void update_notch_coeff();

private:
    PipelineConfig cfg_{};
    size_t n_ch_ = 0;

    // ---- Moving Average (cascaded) ----
    std::vector<uint8_t> ma_inited_;     // size = ma_order
    std::vector<size_t> ma_pos_;         // size = ma_order
    std::vector<float> ma_ring_;         // size = ma_order * ma_win * n_ch
    std::vector<double> ma_sum_;         // size = ma_order * n_ch

    // ---- Exponential Moving Average (cascaded) ----
    std::vector<uint8_t> ema_inited_;    // size = ema_order
    std::vector<float> ema_state_;       // size = ema_order * n_ch

    // ---- Notch biquad (cascaded) ----
    double b0_ = 1, b1_ = 0, b2_ = 0, a1_ = 0, a2_ = 0;
    std::vector<double> nx1_, nx2_, ny1_, ny2_; // size = notch_order * n_ch

    // ---- V/RC + dV/dt (cascaded) ----
    std::vector<size_t> vrc_pos_;        // size = vrc_order * n_ch
    std::vector<uint16_t> vrc_cnt_;      // size = vrc_order * n_ch (clamped to vrc_n)
    std::vector<float> vrc_ring_;        // size = vrc_order * n_ch * vrc_n

    // ---- Bias ----
    BiasCorrector bias_;
};

}

#endif
