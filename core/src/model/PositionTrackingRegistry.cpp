#include "hub/model/PositionTrackingRegistry.h"
#include "hub/model/BruteForce_16x2.h"

#include <functional>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <algorithm>

namespace hub::pt {

struct Entry {
    AlgoInfo info;
    std::function<std::unique_ptr<IAlgorithm>()> factory;
};

static std::vector<Entry>& registry() {
    static std::vector<Entry> reg;
    return reg;
}

static std::mutex& registry_mutex() {
    static std::mutex mu;
    return mu;
}

void register_algorithm(Registration reg) {
    if (reg.info.id.empty()) return;
    if (!reg.factory) return;

    std::lock_guard<std::mutex> lk(registry_mutex());
    auto& r = registry();
    for (const auto& e : r) {
        if (e.info.id == reg.info.id) return;
    }

    Entry e;
    e.info = std::move(reg.info);
    e.factory = std::move(reg.factory);
    r.push_back(std::move(e));
}

static void ensure_registered() {
    static bool done = false;
    if (done) return;
    done = true;

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
                {"rc_r", "RC_R (Ohm)", 1e3, 1e14, 1e8, 0.0, 18, true},
                {"rc_c", "RC_C (F)", 1e-18, 1e-3, 5e-10, 0.0, 18, true},
                {"ema_a", "EMA alpha", 0.0, 1.0, 0.2, 0.01, 4, false},
                {"quiet", "Quiet err thresh", 0.0, 1e6, 0.3, 0.05, 6, false},
                {"xmin", "Grid x min", -1.0, 1.0, -0.03, 0.001, 5, false},
                {"xmax", "Grid x max", -1.0, 1.0, 0.03, 0.001, 5, false},
                {"ymin", "Grid y min", -1.0, 1.0, -0.03, 0.001, 5, false},
                {"ymax", "Grid y max", -1.0, 1.0, 0.03, 0.001, 5, false},
                {"zmin", "Grid z min", -1.0, 1.0, 0.01, 0.001, 5, false},
                {"zmax", "Grid z max", -1.0, 1.0, 0.01, 0.001, 5, false},
                {"step", "Grid step", 1e-6, 0.1, 0.001, 0.0001, 6, false}
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

            double rc_r = a[0];
            double rc_c = a[1];
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

        bool push_sample(uint64_t, const std::vector<float>& sample, Output& out) override {
            if (sample.size() != 16) return false;
            auto r = solver_.update(sample);

            out.valid = r.has_pose;
            out.quiet = r.quiet;
            out.x = r.x;
            out.y = r.y;
            out.z = r.z;
            out.q1 = r.q1;
            out.q2 = r.q2;
            out.err = r.err;

            if (out.valid) {
                double e = out.err;
                if (e < 0.0) e = 0.0;
                out.confidence = 1.0 / (1.0 + e);
            } else {
                out.confidence = 0.0;
            }

            return out.valid;
        }

    private:
        std::vector<double> params_;
        hub::BruteForce_16x2Solver solver_;
    };

    Registration r;
    r.info.id = "BruteForce_16x2";
    r.info.N = 16;
    r.info.M = 2;
    {
        BruteForce_16x2 tmp;
        r.info.params = tmp.params();
        r.info.defaults = tmp.defaults();
    }
    r.factory = []() { return std::make_unique<BruteForce_16x2>(); };
    register_algorithm(std::move(r));
}

std::vector<AlgoInfo> list_algorithms() {
    ensure_registered();
    std::lock_guard<std::mutex> lk(registry_mutex());

    std::vector<AlgoInfo> out;
    out.reserve(registry().size());
    for (auto& e : registry()) out.push_back(e.info);

    std::sort(out.begin(), out.end(), [](const AlgoInfo& a, const AlgoInfo& b) {
        return a.id < b.id;
    });

    return out;
}

AlgoInfo get_algorithm_info(const std::string& id) {
    ensure_registered();
    std::lock_guard<std::mutex> lk(registry_mutex());

    for (auto& e : registry()) {
        if (e.info.id == id) return e.info;
    }
    return AlgoInfo{};
}

std::unique_ptr<IAlgorithm> create_algorithm(const std::string& id) {
    ensure_registered();
    std::lock_guard<std::mutex> lk(registry_mutex());

    for (auto& e : registry()) {
        if (e.info.id == id) return e.factory();
    }
    return {};
}

} // namespace hub::pt
