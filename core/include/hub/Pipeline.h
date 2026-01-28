#pragma once
#include "hub/Frame.h"
#include "hub/filters/MA.h"
#include "hub/filters/EMA.h"
#include "hub/filters/Notch60.h"
#include "hub/filters/Bias.h"
#include "hub/model/Linear.h"
#include <cstddef>
#include <optional>

namespace hub {

struct PipelineConfig {
    bool enable_ma = false;
    bool enable_ema = true;
    bool enable_notch = false;
    bool enable_bias = false;
    bool enable_model = true;

    size_t ma_win = 5;
    float ema_alpha = 0.2f;

    double fs_hz = 200.0;
    double notch_f0 = 60.0;
    double notch_q = 30.0;

    float model_bias = 0.0f;
};

struct PipelineOut {
    Frame frame;
    float model_out = 0.0f;
    bool model_valid = false;
};

class Pipeline {
public:
    void reset();
    void set_config(const PipelineConfig& cfg);
    const PipelineConfig& config() const { return cfg_; }

    void ensure_initialized(size_t n_ch);

    void begin_bias_capture(size_t frames);
    bool bias_capturing() const { return bias_.capturing(); }
    bool bias_has() const { return bias_.has_bias(); }
    const std::vector<float>& bias_vec() const { return bias_.bias(); }

    void set_model_weights(const std::vector<float>& w);

    PipelineOut process(uint64_t t_ns, const std::vector<float>& x_in);

private:
    PipelineConfig cfg_;
    size_t n_ch_ = 0;

    MAFilter ma_;
    EMAFilter ema_;
    NotchBiquad notch_;
    BiasCorrector bias_;
    LinearModel model_;
};

}
