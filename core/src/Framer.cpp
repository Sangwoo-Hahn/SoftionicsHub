#include "hub/Framer.h"

namespace hub {

std::vector<std::string> LineFramer::push(std::string_view chunk) {
    buf_.append(chunk.data(), chunk.size());

    std::vector<std::string> out;
    size_t start = 0;

    // Accept any of: "\n", "\r", "\r\n" as line terminators.
    // This makes Serial streams robust: some devices emit CR-only.
    for (;;) {
        size_t nl = buf_.find_first_of("\r\n", start);
        if (nl == std::string::npos) break;

        size_t end = nl;

        if (end > start) out.emplace_back(buf_.substr(start, end - start));
        else out.emplace_back(std::string());

        // Consume delimiter (\r\n treated as a single newline)
        size_t adv = 1;
        if (buf_[nl] == '\r' && (nl + 1) < buf_.size() && buf_[nl + 1] == '\n') {
            adv = 2;
        }
        start = nl + adv;
    }

    if (start > 0) buf_.erase(0, start);
    return out;
}

void LineFramer::clear() {
    buf_.clear();
}

}
