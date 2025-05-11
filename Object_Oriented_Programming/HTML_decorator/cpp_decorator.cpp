#include "cpp_decorator.hpp"

std::string CPPDecorator::write(const std::string& str) {
    return Decorate(Colorize(str, key_words));
}

std::string CPPDecorator::Colorize(const std::string& in_str, const std::map<std::string, Color>& words) {
    if (in_str.empty())
        return in_str;

    std::string result = in_str;

    for (auto& it : words) {
        std::stringstream change;
        change << "<font color=" << it.second << ">" << it.first << "</font>";
        size_t pos = 0;
        auto at = result.find(it.first, pos);
        while (at < result.length()) {
            result.replace(at, it.first.size(), change.str());
            pos = at + change.str().length();
            at = result.find(it.first, pos);
        }
        change.clear();
    }
    return result;
}
