#pragma once
#include <string>
#include <string_view>
#include <vector>

namespace hub {

class LineFramer {
public:
    std::vector<std::string> push(std::string_view chunk);
    void clear();

private:
    std::string buf_;
};

}
