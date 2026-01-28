#pragma once
#include <optional>
#include <string_view>
#include <vector>

namespace hub {

class CsvFloatParser {
public:
    std::optional<std::vector<float>> parse_line(std::string_view line) const;
};

}
