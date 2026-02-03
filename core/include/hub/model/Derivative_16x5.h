#pragma once
#include "hub/model/PositionTrackingRegistry.h"

#include <array>
#include <vector>
#include <string>
#include <cstdint>
#include <cmath>
#include <algorithm>

namespace hub::pt {

class Derivative_16x5 final : public IAlgorithm {
public:
    Derivative_16x5();

    const std::string& id() const override;
    int N() const override;
    int M() const override;

    std::vector<hub::pt::ParamDesc> params() const override;
    std::vector<double> defaults() const override;
    void set_params(const std::vector<double>& values) override;

    void reset() override;

    bool push_sample(uint64_t t_ns, const std::vector<float>& sample, hub::pt::Output& out) override;

private:
    static constexpr int kN = 16;
    static constexpr int kM = 5;
    static constexpr int kEmaMaxDegree = 5;

    const std::array<float, kN>& at_age(int age) const;

    void fill_output_quiet(hub::pt::Output& out) const;

    std::string id_;

    std::array<std::array<float, kN>, kM> buf_;
    int count_;
    int head_;

    uint64_t last_t_ns_;

    int m_effective_;

    double ema_alpha_;
    int ema_degree_;
    double range_gain_;
    double noise_round_;

    std::array<double, kN> sx_;
    std::array<double, kN> sy_;

    bool has_bounds_;
    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;

    bool ema_inited_;
    std::array<double, kEmaMaxDegree> x_ema_;
    std::array<double, kEmaMaxDegree> y_ema_;
};

}
