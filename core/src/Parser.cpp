#include "hub/Parser.h"
#include <cstdlib>

namespace hub {

static inline void skip_ws(const char*& p, const char* e) {
    while (p < e && (*p == ' ' || *p == '\t')) ++p;
}

static inline bool is_number_start(char c) {
    return (c >= '0' && c <= '9') || c == '+' || c == '-' || c == '.';
}

std::optional<std::vector<float>> CsvFloatParser::parse_line(std::string_view line) const {
    const char* p = line.data();
    const char* e = p + line.size();

    std::vector<float> vals;
    vals.reserve(64);

    for (;;) {
        // Allow leading separators (",,,,1,2"), and allow whitespace-separated formats.
        // This is especially common on Serial streams (e.g. Arduino Serial Plotter uses spaces/tabs).
        for (;;) {
            skip_ws(p, e);
            if (p >= e) break;
            if (*p == ',' || *p == ';' || *p == '|') {
                ++p;
                continue;
            }
            break;
        }
        if (p >= e) break;

        char* endp = nullptr;
        float v = std::strtof(p, &endp);
        if (endp == p) return std::nullopt;

        vals.push_back(v);
        p = endp;

        // Skip whitespace after a value.
        skip_ws(p, e);
        if (p >= e) break;

        // Common explicit separators.
        if (*p == ',' || *p == ';' || *p == '|') {
            ++p;
            continue;
        }

        // Whitespace-separated format: if the next token looks like a number, keep parsing.
        // (We already consumed whitespace above.)
        if (is_number_start(*p)) {
            continue;
        }

        // Otherwise, unknown / stray characters.
        return std::nullopt;
    }

    if (vals.empty()) return std::nullopt;
    return vals;
}

}
