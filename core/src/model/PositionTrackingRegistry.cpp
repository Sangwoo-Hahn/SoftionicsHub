#include "hub/model/PositionTrackingRegistry.h"
#include "hub/model/BruteForce_16x2.h"

#include <functional>
#include <vector>
#include <string>
#include <memory>

namespace hub::pt {

struct Entry {
    AlgoInfo info;
    std::function<std::unique_ptr<IAlgorithm>()> factory;
};

static std::vector<Entry>& registry() {
    static std::vector<Entry> reg;
    return reg;
}

static void ensure_registered() {
    auto& reg = registry();
    if (!reg.empty()) return;

    // -------------------------
    // BruteForce_16x2
    // -------------------------
    class BruteForce_16x2 final : public IAlgorithm {
    public:
        BruteForce_16x2() {
            reset();
            set_params(defaults());
        }

        const std::string& id() const override {
            static std::string s = "BruteForce_16x2";
            return s;
        }
        int N() const override { return 16; }
        int M() const override { return 2; }

        std::vector<ParamDesc> params() const override {
            return {
                {"rc_r",   "RC_R (Ohm)",        1e3,   1e14,  1e8},
                {"rc_c",   "RC_C (F)",          1e-18, 1e-3,  5e-10},
                {"ema_a",  "EMA alpha",         0.0,   1.0,   0.2},
                {"quiet",  "Quiet err thresh",  0.0,   1e6,   0.3},

                {"xmin", "Grid x min", -1.0, 1.0, -0.06},
                {"xmax", "Grid x max", -1.0, 1.0,  0.06},
                {"ymin", "Grid y min", -1.0, 1.0, -0.06},
                {"ymax", "Grid y max", -1.0, 1.0,  0.06},
                {"zmin", "Grid z min", -1.0, 1.0,  0.01},
                {"zmax", "Grid z max", -1.0, 1.0,  0.10},
                {"step", "Grid step",  1e-6, 0.1,  0.01},
            };
        }

        std::vector<double> defaults() const override {
            auto p = params();
            std::vector<double> d;
            d.reserve(p.size());
            for (auto& x : p) d.push_back(x.defv);
            return d;
        }

        void set_params(const std::vector<double>& v) override {
            auto d = defaults();
            std::vector<double> a = v;
            if (a.size() < d.size()) a.resize(d.size(), 0.0);

            double rc_r  = a[0];
            double rc_c  = a[1];
            double ema_a = a[2];
            double quiet = a[3];

            double xmin = a[4], xmax = a[5];
            double ymin = a[6], ymax = a[7];
            double zmin = a[8], zmax = a[9];
            double step = a[10];

            solver_.set_params(rc_r, rc_c, ema_a, quiet);
            solver_.set_grid(xmin, xmax, ymin, ymax, zmin, zmax, step);

            params_ = a;
        }

        void reset() override {
            solver_.reset();
        }

        bool push_sample(const std::vector<float>& sample, Output& out) override {
            if (sample.size() != 16) return false;
            auto r = solver_.update(sample);

            out.valid = r.has_pose;
            out.quiet = r.quiet;
            out.x = r.x; out.y = r.y; out.z = r.z;
            out.q1 = r.q1; out.q2 = r.q2;
            out.err = r.err;
            return out.valid;
        }

    private:
        std::vector<double> params_;
        hub::BruteForce_16x2Solver solver_;
    };

    Entry e;
    e.info.id = "BruteForce_16x2";
    e.info.N = 16;
    e.info.M = 2;
    {
        BruteForce_16x2 tmp;
        e.info.params = tmp.params();
        e.info.defaults = tmp.defaults();
    }
    e.factory = []() { return std::make_unique<BruteForce_16x2>(); };
    reg.push_back(std::move(e));

    // 여기에 새 알고리즘들을 계속 추가하면 됨
}

std::vector<AlgoInfo> list_algorithms() {
    ensure_registered();
    std::vector<AlgoInfo> out;
    for (auto& e : registry()) out.push_back(e.info);
    return out;
}

AlgoInfo get_algorithm_info(const std::string& id) {
    ensure_registered();
    for (auto& e : registry()) {
        if (e.info.id == id) return e.info;
    }
    return AlgoInfo{};
}

std::unique_ptr<IAlgorithm> create_algorithm(const std::string& id) {
    ensure_registered();
    for (auto& e : registry()) {
        if (e.info.id == id) return e.factory();
    }
    return {};
}

} // namespace hub::pt
