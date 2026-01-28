#include "hub/Pipeline.h"

namespace hub {

void Pipeline::reset() {
    cfg_ = PipelineConfig{};
    n_ch_ = 0;
    ma_.reset();
    ema_.reset();
    notch_.reset();
    bias_.reset();
    model_.reset();
}

void Pipeline::set_config(const PipelineConfig& cfg) {
    cfg_ = cfg;
    if (n_ch_ == 0) return;

    if (!ma_.ready() || ma_.win_len() != cfg_.ma_win) {
        ma_.configure(n_ch_, cfg_.ma_win);
    }

    if (!ema_.ready()) {
        ema_.configure(n_ch_, cfg_.ema_alpha);
    } else {
        ema_.set_alpha(cfg_.ema_alpha);
    }

    if (!notch_.ready()) {
        notch_.configure(n_ch_, cfg_.fs_hz, cfg_.notch_f0, cfg_.notch_q);
    } else {
        notch_.set_params(cfg_.fs_hz, cfg_.notch_f0, cfg_.notch_q);
    }

    if (bias_.bias().size() != n_ch_) {
        bias_.configure(n_ch_);
    }

    if (!model_.ready()) {
        model_.configure(n_ch_);
    }
    model_.set_bias(cfg_.model_bias);
}

void Pipeline::ensure_initialized(size_t n_ch) {
    if (n_ch_ == n_ch && n_ch_ > 0) return;
    n_ch_ = n_ch;

    ma_.configure(n_ch_, cfg_.ma_win);
    ema_.configure(n_ch_, cfg_.ema_alpha);
    notch_.configure(n_ch_, cfg_.fs_hz, cfg_.notch_f0, cfg_.notch_q);
    bias_.configure(n_ch_);
    model_.configure(n_ch_);
    model_.set_bias(cfg_.model_bias);
}

void Pipeline::begin_bias_capture(size_t frames) {
    if (n_ch_ == 0) return;
    bias_.begin_capture(frames);
}

void Pipeline::set_model_weights(const std::vector<float>& w) {
    if (n_ch_ == 0) return;
    model_.set_weights(w);
}

PipelineOut Pipeline::process(uint64_t t_ns, const std::vector<float>& x_in) {
    PipelineOut out;
    out.frame.t_ns = t_ns;
    out.frame.x = x_in;

    if (n_ch_ == 0) ensure_initialized(out.frame.x.size());
    if (out.frame.x.size() != n_ch_) return out;

    if (bias_.capturing()) bias_.update_capture(out.frame.x);

    if (cfg_.enable_bias) bias_.apply_inplace(out.frame.x);
    if (cfg_.enable_notch) notch_.process_inplace(out.frame.x);
    if (cfg_.enable_ma) ma_.process_inplace(out.frame.x);
    if (cfg_.enable_ema) ema_.process_inplace(out.frame.x);

    if (cfg_.enable_model && model_.ready()) {
        out.model_out = model_.eval(out.frame.x);
        out.model_valid = true;
    }

    return out;
}

}
