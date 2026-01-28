#include "hub/Framer.h"

namespace hub {

std::vector<std::string> LineFramer::push(std::string_view chunk) {
    buf_.append(chunk.data(), chunk.size());

    std::vector<std::string> out;
    size_t start = 0;

    for (;;) {
        size_t nl = buf_.find('\n', start);
        if (nl == std::string::npos) break;

        size_t end = nl;
        if (end > 0 && buf_[end - 1] == '\r') end--;

        if (end > start) out.emplace_back(buf_.substr(start, end - start));
        else out.emplace_back(std::string());

        start = nl + 1;
    }

    if (start > 0) buf_.erase(0, start);
    return out;
}

void LineFramer::clear() {
    buf_.clear();
}

}
