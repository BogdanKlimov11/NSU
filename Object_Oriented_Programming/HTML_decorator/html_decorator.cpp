#include "html_decorator.hpp"

std::string HTMLDecorator::replaceCharacters(const std::string& str) {
    if (str.empty())
        return str;

    std::string result = str;
    for (auto& special_symbol : special_symbols) {
        size_t pos = 0;
        auto symbol_at = result.find(special_symbol.first, pos);
        while (symbol_at < result.length()) {
            result = result.replace(symbol_at, special_symbol.first.size(), special_symbol.second);
            pos = symbol_at + special_symbol.second.length();
            symbol_at = result.find(special_symbol.first, pos);
        }
    }
    return result;
}

std::string HTMLDecorator::write(const std::string& str) {
    std::string str_n = replaceCharacters(str);
    return "<html>\n<body>\n<pre>" + Decorate(str_n) + "\n</pre>\n</body>\n</html>";
}
