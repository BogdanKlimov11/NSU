#include "line_decorator.hpp"

std::string LineDecorator::write(const std::string& str) {
    if (str.empty())
        return str;

    int count_lines = 0;
    std::stringstream result_stream, str_stream1(str), str_stream2(str);
    std::string line;

    while (std::getline(str_stream1, line)) {
        ++count_lines;
    }

    int i = 1;
    while (std::getline(str_stream2, line)) {
        result_stream << std::setw(std::to_string(count_lines).length())
                      << i << "| " << line << std::endl;
        ++i;
    }

    return result_stream.str();
}
