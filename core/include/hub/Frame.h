#pragma once
#include <cstdint>
#include <vector>

namespace hub {

struct Frame {
    uint64_t t_ns = 0;
    std::vector<float> x;
};

}
