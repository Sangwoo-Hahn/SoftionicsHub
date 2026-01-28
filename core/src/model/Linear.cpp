#include "hub/model/Linear.h"
#include <fstream>
#include <sstream>
#include <cstdlib>

namespace hub {

void LinearModel::reset() {
    ready_ = false;
    n_ch_ = 0;
    w_.clear();
    b_ = 0.0f;
}

void LinearModel::configure(size_t n_ch) {
    if (n_ch == 0) {
        reset();
        return;
    }
    if (ready_ && n_ch_ == n_ch && w_.size() == n_ch_) {
        return;
    }
    n_ch_ = n_ch;
    w_.assign(n_ch_, 0.0f);
    b_ = 0.0f;
    ready_ = true;
}

void LinearModel::set_bias(float b) {
    b_ = b;
}

void LinearModel::set_weights(const std::vector<float>& w) {
    if (!ready_) return;
    if (w.size() != n_ch_) return;
    w_ = w;
}

float LinearModel::eval(const std::vector<float>& x) const {
    if (!ready_) return 0.0f;
    if (x.size() != n_ch_) return 0.0f;
    double acc = static_cast<double>(b_);
    for (size_t i = 0; i < n_ch_; ++i) acc += static_cast<double>(w_[i]) * static_cast<double>(x[i]);
    return static_cast<float>(acc);
}

static inline void trim(std::string& s) {
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t')) s.pop_back();
    size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
    if (i > 0) s.erase(0, i);
}

std::optional<std::vector<float>> load_weights_csv_1line(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs) return std::nullopt;

    std::string line;
    if (!std::getline(ifs, line)) return std::nullopt;
    trim(line);
    if (line.empty()) return std::nullopt;

    std::vector<float> w;
    w.reserve(64);

    const char* p = line.data();
    const char* e = p + line.size();

    for (;;) {
        while (p < e && (*p == ' ' || *p == '\t')) ++p;
        if (p >= e) break;

        char* endp = nullptr;
        float v = std::strtof(p, &endp);
        if (endp == p) return std::nullopt;

        w.push_back(v);
        p = endp;

        while (p < e && (*p == ' ' || *p == '\t')) ++p;
        if (p >= e) break;

        if (*p == ',') {
            ++p;
            continue;
        }
        return std::nullopt;
    }

    if (w.empty()) return std::nullopt;
    return w;
}

}
