#ifndef HUB_MODEL_POSITIONTRACKINGREGISTRY_H
#define HUB_MODEL_POSITIONTRACKINGREGISTRY_H

#include <vector>
#include <string>
#include <memory>
#include <array>
#include <cstdint>
#include <functional>

namespace hub::pt {

struct Output {
    bool valid = false;
    bool quiet = false;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double confidence = 0.0;
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
    double step = 0.0;
    int decimals = -1;
    bool scientific = false;
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

    virtual bool push_sample(uint64_t t_ns, const std::vector<float>& sample, Output& out) = 0;
};

template<int NC, int MC>
class SlidingWindowAlgorithm : public IAlgorithm {
public:
    static_assert(NC > 0, "NC must be > 0");
    static_assert(MC > 0, "MC must be > 0");

    int N() const override { return NC; }
    int M() const override { return MC; }

    struct WindowView {
        const std::array<std::array<float, NC>, MC>* ring = nullptr;
        int start = 0;

        const std::array<float, NC>& at(int k) const {
            return (*ring)[(start + k) % MC];
        }

        const float* data(int k) const {
            return at(k).data();
        }
    };

    bool push_sample(uint64_t t_ns, const std::vector<float>& sample, Output& out) override {
        if ((int)sample.size() != NC) return false;

        auto& dst = ring_[pos_];
        for (int i = 0; i < NC; ++i) dst[i] = sample[(size_t)i];

        pos_ = (pos_ + 1) % MC;
        if (filled_ < MC) ++filled_;
        if (filled_ < MC) return false;

        WindowView w;
        w.ring = &ring_;
        w.start = pos_;

        out = compute(w, params_, t_ns);
        return out.valid;
    }

    void set_params(const std::vector<double>& values) override {
        params_ = values;
    }

    void reset() override {
        pos_ = 0;
        filled_ = 0;
    }

protected:
    virtual Output compute(const WindowView& w, const std::vector<double>& params, uint64_t t_ns) = 0;

    std::vector<double> params_;

private:
    std::array<std::array<float, NC>, MC> ring_{};
    int pos_ = 0;
    int filled_ = 0;
};

struct Registration {
    AlgoInfo info;
    std::function<std::unique_ptr<IAlgorithm>()> factory;
};

void register_algorithm(Registration reg);

template<class T>
Registration make_registration() {
    T tmp;
    Registration r;
    r.info.id = tmp.id();
    r.info.N = tmp.N();
    r.info.M = tmp.M();
    r.info.params = tmp.params();
    r.info.defaults = tmp.defaults();
    r.factory = []() { return std::make_unique<T>(); };
    return r;
}

#define HUB_PT_REGISTER_ALGORITHM(AlgoClass) \
namespace { \
struct AlgoClass##_AutoReg { \
    AlgoClass##_AutoReg() { \
        ::hub::pt::register_algorithm(::hub::pt::make_registration<AlgoClass>()); \
    } \
}; \
static AlgoClass##_AutoReg g_##AlgoClass##_AutoReg; \
}

std::vector<AlgoInfo> list_algorithms();
AlgoInfo get_algorithm_info(const std::string& id);
std::unique_ptr<IAlgorithm> create_algorithm(const std::string& id);

} // namespace hub::pt

#endif
