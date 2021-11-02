#ifndef NAMEOF_H
#define NAMEOF_H


#define getNameOf(x) bipedlab::nameof_<0>(#x, sizeof(x))


#include <string>
#include <regex>
#include <stdexcept>

namespace bipedlab {
    template<int a>
    std::string nameof_(const std::string &x, std::size_t)
    {
        std::regex regex("^&?([_a-zA-Z]\\w*(->|\\.|::))*([_a-zA-Z]\\w*)$");
        std::smatch match;
        if (std::regex_match(x, match, regex)) {
            if (match.size() == 4) {
                return match[3];
            }
        }
        throw std::logic_error(
            "A bad expression x in nameof(x). The expression is \"" + x + "\"."
        );
    }
}

#endif /* #ifndef NAMEOF_H */
