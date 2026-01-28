#include "hub/Parser.h"
#include <cstdlib>

namespace hub {

static inline void skip_ws(const char*& p, const char* e) {
    while (p < e && (*p == ' ' || *p == '\t')) ++p;
}

std::optional<std::vector<float>> CsvFloatParser::parse_line(std::string_view line) const {
    const char* p = line.data();
    const char* e = p + line.size();

    std::vector<float> vals;
    vals.reserve(64);

    for (;;) {
        skip_ws(p, e);
        if (p >= e) break;

        char* endp = nullptr;
        float v = std::strtof(p, &endp);
        if (endp == p) return std::nullopt;

        vals.push_back(v);
        p = endp;

        skip_ws(p, e);
        if (p >= e) break;

        if (*p == ',') {
            ++p;
            continue;
        }

        return std::nullopt;
    }

    if (vals.empty()) return std::nullopt;
    return vals;
}

}
