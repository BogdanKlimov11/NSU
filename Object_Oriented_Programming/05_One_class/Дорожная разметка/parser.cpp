#include "parser.hpp"

std::pair<std::vector<Line>, bool> Parser::getLines() {
    std::string mark, line;
    double lenght;
    std::vector<Line> result;
    while (std::getline(is, line)) {
        std::istringstream iss(line);
        if (!(iss >> lenght)) continue;
        if (!(iss >> mark)) continue;
        auto converted = convert(mark);
        result.push_back(Line{ lenght, converted });
    }
    if (!result.empty()) {
        return std::make_pair(result, true);
    }
    return std::make_pair(result, false);
}
