#pragma once
#include <optional>
#include <string_view>
#include <vector>

namespace hub {

// Input format for Serial / CSV-like streams.
//
// - Auto: try Decimal first; if it fails, attempt to detect and decode Hex/Binary.
// - Decimal: CSV / whitespace-separated decimal floats or ints.
// - Hex: tokens interpreted as hexadecimal integers (e.g. "0x1A", "1A").
// - Binary: tokens interpreted as binary integers (e.g. "0b1010", "1010").
// - Int16: tokens parsed as integers (Auto base detect) and converted to signed int16.
enum class InputFormat : int {
    Auto    = 0,
    Decimal = 1,
    Hex     = 2,
    Binary  = 3,
    // Parse tokens as integers (Auto base detection) and reinterpret as signed int16
    // using two's-complement: range [-32768, 32767].
    Int16   = 4,
};

class CsvFloatParser {
public:
    // Backwards compatible: defaults to Auto.
    std::optional<std::vector<float>> parse_line(std::string_view line) const {
        return parse_line(line, InputFormat::Auto);
    }

    std::optional<std::vector<float>> parse_line(std::string_view line, InputFormat fmt) const;
};

}
