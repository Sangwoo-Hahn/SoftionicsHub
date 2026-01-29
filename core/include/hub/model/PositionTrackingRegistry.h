#ifndef HUB_MODEL_POSITIONTRACKINGREGISTRY_H
#define HUB_MODEL_POSITIONTRACKINGREGISTRY_H

#include <vector>
#include <string>
#include <memory>
#include <array>
#include <deque>

namespace hub::pt {

struct Output {
    bool valid = false;
    bool quiet = false;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double q1 = 0.0;
    double q2 = 0.0;
    double err = 0.0;
};

struct ParamDesc {
    std::string key;
    std::string label;
    double minv = 0.0;
    double maxv = 0.0;
    double defv = 0.0;
};

struct AlgoInfo {
    std::string id;
    int N = 0;
    int M = 0;
    std::vector<ParamDesc> params;
    std::vector<double> defaults;
};

class IAlgorithm {
public:
    virtual ~IAlgorithm() = default;

    virtual const std::string& id() const = 0;
    virtual int N() const = 0;
    virtual int M() const = 0;

    virtual std::vector<ParamDesc> params() const = 0;
    virtual std::vector<double> defaults() const = 0;

    virtual void set_params(const std::vector<double>& values) = 0;
    virtual void reset() = 0;

    // sample은 "필터/바이어스 적용된" 1프레임(N채널)
    virtual bool push_sample(const std::vector<float>& sample, Output& out) = 0;
};

// -------------------------
// Sliding Window (N채널, M히스토리) 기반 알고리즘 베이스
// -------------------------
template<int N, int M>
class SlidingWindowAlgorithm : public IAlgorithm {
public:
    int N() const override { return N; }
    int M() const override { return M; }

    bool push_sample(const std::vector<float>& sample, Output& out) override {
        if ((int)sample.size() != N) return false;

        std::array<float, N> s{};
        for (int i = 0; i < N; ++i) s[i] = sample[i];

        if ((int)hist_.size() == M) hist_.pop_front();
        hist_.push_back(s);

        if ((int)hist_.size() < M) return false;

        std::array<std::array<float, N>, M> H{};
        int k = 0;
        for (const auto& a : hist_) {
            H[k++] = a;
        }
        out = compute(H, params_);
        return out.valid;
    }

    void set_params(const std::vector<double>& values) override {
        params_ = values;
    }

    void reset() override {
        hist_.clear();
    }

protected:
    virtual Output compute(const std::array<std::array<float, N>, M>& H,
                           const std::vector<double>& params) = 0;

    std::vector<double> params_;

private:
    std::deque<std::array<float, N>> hist_;
};

// -------------------------
// Registry API
// -------------------------
std::vector<AlgoInfo> list_algorithms();
AlgoInfo get_algorithm_info(const std::string& id);
std::unique_ptr<IAlgorithm> create_algorithm(const std::string& id);

} // namespace hub::pt

#endif
