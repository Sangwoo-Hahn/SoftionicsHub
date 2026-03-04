#include "hub/Parser.h"
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <string>

#include <algorithm>

namespace hub {

static inline void skip_ws(const char*& p, const char* e) {
    while (p < e && (*p == ' ' || *p == '\t')) ++p;
}

static inline bool is_number_start(char c) {
    return (c >= '0' && c <= '9') || c == '+' || c == '-' || c == '.';
}

static inline bool is_sep(char c) {
    return c == ',' || c == ';' || c == '|' || c == ' ' || c == '\t';
}

static inline bool is_hex_digit(char c) {
    return (c >= '0' && c <= '9') ||
           (c >= 'a' && c <= 'f') ||
           (c >= 'A' && c <= 'F');
}

static inline bool is_bin_digit(char c) {
    return c == '0' || c == '1';
}

static std::optional<std::vector<float>> parse_decimal_impl(std::string_view line) {
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

static void split_tokens(std::string_view line, std::vector<std::string_view>& out) {
    out.clear();
    const char* b = line.data();
    const char* e = b + line.size();

    const char* p = b;
    for (;;) {
        while (p < e && is_sep(*p)) ++p;
        if (p >= e) break;
        const char* s = p;
        while (p < e && !is_sep(*p)) ++p;
        const char* t = p;
        if (t > s) out.emplace_back(s, (size_t)(t - s));
    }
}

static std::optional<std::vector<float>> parse_hex_impl(std::string_view line) {
    std::vector<std::string_view> toks;
    toks.reserve(64);
    split_tokens(line, toks);
    if (toks.empty()) return std::nullopt;

    std::vector<float> vals;
    vals.reserve(toks.size());

    for (auto tsv : toks) {
        if (tsv.empty()) continue;

        // Trim surrounding whitespace (defensive; split_tokens should already remove it)
        while (!tsv.empty() && (tsv.front() == ' ' || tsv.front() == '\t')) tsv.remove_prefix(1);
        while (!tsv.empty() && (tsv.back() == ' ' || tsv.back() == '\t')) tsv.remove_suffix(1);
        if (tsv.empty()) continue;

        // Optional 0x/0X prefix.
        if (tsv.size() >= 2 && tsv[0] == '0' && (tsv[1] == 'x' || tsv[1] == 'X')) {
            tsv.remove_prefix(2);
        }
        if (tsv.empty()) return std::nullopt;

        // Allow a leading + sign. (No negative hex in Auto mode without explicit '-')
        if (tsv.front() == '+') {
            tsv.remove_prefix(1);
            if (tsv.empty()) return std::nullopt;
        }

        // Validate: hex digits only.
        for (char c : tsv) {
            if (!is_hex_digit(c)) return std::nullopt;
        }

        std::string tok(tsv);
        char* endp = nullptr;
        unsigned long v = std::strtoul(tok.c_str(), &endp, 16);
        if (!endp || endp == tok.c_str() || *endp != '\0') return std::nullopt;
        vals.push_back((float)v);
    }

    if (vals.empty()) return std::nullopt;
    return vals;
}

static std::optional<std::vector<float>> parse_binary_impl(std::string_view line) {
    std::vector<std::string_view> toks;
    toks.reserve(64);
    split_tokens(line, toks);
    if (toks.empty()) return std::nullopt;

    std::vector<float> vals;
    vals.reserve(toks.size());

    for (auto tsv : toks) {
        if (tsv.empty()) continue;

        // Optional 0b/0B prefix.
        if (tsv.size() >= 2 && tsv[0] == '0' && (tsv[1] == 'b' || tsv[1] == 'B')) {
            tsv.remove_prefix(2);
        }
        if (tsv.empty()) return std::nullopt;

        // Validate: binary digits only.
        for (char c : tsv) {
            if (!is_bin_digit(c)) return std::nullopt;
        }

        std::string tok(tsv);
        char* endp = nullptr;
        unsigned long v = std::strtoul(tok.c_str(), &endp, 2);
        if (!endp || endp == tok.c_str() || *endp != '\0') return std::nullopt;
        vals.push_back((float)v);
    }

    if (vals.empty()) return std::nullopt;
    return vals;
}

static inline bool token_has_hex_letter(std::string_view tok) {
    for (char c : tok) {
        if ((c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) return true;
    }
    return false;
}

static inline bool token_is_binary_like(std::string_view tok) {
    // Length threshold to avoid mis-detecting normal decimals like "10,11".
    // Typical binary dumps are 8/16/32 bits.
    if (tok.size() < 8) return false;
    for (char c : tok) {
        if (!is_bin_digit(c)) return false;
    }
    return true;
}

static inline int32_t u16_to_i16(uint16_t w) {
    // Avoid implementation-defined casts from unsigned->signed by doing explicit mapping.
    return (w & 0x8000u) ? (int32_t)w - 0x10000 : (int32_t)w;
}

static std::optional<std::vector<float>> parse_int16_impl(std::string_view line) {
    std::vector<std::string_view> toks;
    toks.reserve(64);
    split_tokens(line, toks);
    if (toks.empty()) return std::nullopt;

    std::vector<float> vals;
    vals.reserve(toks.size());

    for (auto tsv : toks) {
        if (tsv.empty()) continue;

        // Parse sign, then detect base (10/16/2) similarly to Auto.
        bool neg = false;
        std::string_view t = tsv;

        if (!t.empty() && (t.front() == '+' || t.front() == '-')) {
            neg = (t.front() == '-');
            t.remove_prefix(1);
            if (t.empty()) return std::nullopt;
        }

        int base = 10;
        std::string_view body = t;

        if (t.size() >= 2 && t[0] == '0' && (t[1] == 'x' || t[1] == 'X')) {
            base = 16;
            body = t.substr(2);
        } else if (t.size() >= 2 && t[0] == '0' && (t[1] == 'b' || t[1] == 'B')) {
            base = 2;
            body = t.substr(2);
        } else if (token_has_hex_letter(t)) {
            base = 16;
            body = t;
        } else if (token_is_binary_like(t)) {
            base = 2;
            body = t;
        } else {
            base = 10;
            body = t;
        }

        if (body.empty()) return std::nullopt;

        uint16_t w = 0;

        if (base == 10) {
            // Decimal signed integer.
            std::string tok(tsv);
            char* endp = nullptr;
            errno = 0;
            long long v = std::strtoll(tok.c_str(), &endp, 10);
            if (!endp || endp == tok.c_str() || *endp != '\0') return std::nullopt;
            if (errno == ERANGE) return std::nullopt;

            uint64_t uv = (uint64_t)(int64_t)v; // defined: modulo 2^64 if v<0
            w = (uint16_t)(uv & 0xFFFFu);
        } else if (base == 16 || base == 2) {
            // Hex/Binary unsigned integer, with optional leading sign handled above.
            std::string tok(body);
            char* endp = nullptr;
            errno = 0;
            unsigned long long u = std::strtoull(tok.c_str(), &endp, base);
            if (!endp || endp == tok.c_str() || *endp != '\0') return std::nullopt;
            if (errno == ERANGE) return std::nullopt;

            uint64_t uv = (uint64_t)u;
            if (neg) uv = 0ULL - uv; // defined modulo arithmetic
            w = (uint16_t)(uv & 0xFFFFu);
        } else {
            return std::nullopt;
        }

        int32_t s = u16_to_i16(w);
        vals.push_back((float)s);
    }

    if (vals.empty()) return std::nullopt;
    return vals;
}

static bool looks_like_hex(std::string_view line) {
    // Heuristics:
    // - Any 0x/0X prefix
    // - Any A..F/a..f letter
    // - Any standalone 'x'/'X'
    bool has_hex_letter = false;
    bool has_0x = false;
    for (size_t i = 0; i < line.size(); ++i) {
        char c = line[i];
        if ((c == 'x' || c == 'X') && i > 0 && line[i - 1] == '0') has_0x = true;
        if ((c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) has_hex_letter = true;
    }
    return has_0x || has_hex_letter;
}

static bool looks_like_binary(std::string_view line) {
    // Heuristics:
    // - Any 0b/0B prefix
    // - Or tokens that look like a bit-field (only 0/1) with a "byte-ish" length.
    bool has_0b = false;
    for (size_t i = 0; i < line.size(); ++i) {
        char c = line[i];
        if ((c == 'b' || c == 'B') && i > 0 && line[i - 1] == '0') {
            has_0b = true;
            break;
        }
    }
    if (has_0b) return true;

    std::vector<std::string_view> toks;
    toks.reserve(64);
    split_tokens(line, toks);
    for (auto t : toks) {
        // Length threshold to avoid mis-detecting normal decimals like "10,11".
        // Typical binary dumps are 8/16/32 bits.
        if (t.size() < 8) continue;
        bool ok = true;
        for (char c : t) {
            if (!is_bin_digit(c)) { ok = false; break; }
        }
        if (ok) return true;
    }
    return false;
}

static std::optional<std::vector<float>> extract_decimal_numbers(std::string_view line) {
    // Fallback for streams that include labels or wrappers.
    // Example: "ax: 1.0, ay: 2.0".
    const char* b = line.data();
    const char* e = b + line.size();
    const char* p = b;

    std::vector<float> vals;
    vals.reserve(64);

    while (p < e) {
        // Find next number-like start.
        while (p < e && !is_number_start(*p)) ++p;
        if (p >= e) break;

        // Guard: don't treat "0x" or "0b" as a decimal 0.
        if (*p == '0' && (p + 1) < e && (p[1] == 'x' || p[1] == 'X' || p[1] == 'b' || p[1] == 'B')) {
            p += 2;
            continue;
        }

        char* endp = nullptr;
        float v = std::strtof(p, &endp);
        if (!endp || endp == p) {
            ++p;
            continue;
        }

        vals.push_back(v);
        p = endp;
    }

    if (vals.empty()) return std::nullopt;
    return vals;
}

std::optional<std::vector<float>> CsvFloatParser::parse_line(std::string_view line, InputFormat fmt) const {
    if (fmt == InputFormat::Decimal) {
        // Decimal-only strict mode.
        if (auto v = parse_decimal_impl(line)) return v;
        return extract_decimal_numbers(line);
    }

    if (fmt == InputFormat::Hex) {
        if (auto v = parse_hex_impl(line)) return v;
        return std::nullopt;
    }

    if (fmt == InputFormat::Binary) {
        if (auto v = parse_binary_impl(line)) return v;
        return std::nullopt;
    }

    if (fmt == InputFormat::Int16) {
        if (auto v = parse_int16_impl(line)) return v;
        return std::nullopt;
    }


    // Auto
    // If it strongly looks like a bit-field, prefer decoding as binary.
    if (looks_like_binary(line)) {
        if (auto v = parse_binary_impl(line)) return v;
    }

    // If it looks like hex, try hex before decimal.
    if (looks_like_hex(line)) {
        if (auto v = parse_hex_impl(line)) return v;
    }

    // Default / most common.
    if (auto v = parse_decimal_impl(line)) return v;

    // As a last resort, try to salvage decimal numbers embedded in text.
    return extract_decimal_numbers(line);
}

}
