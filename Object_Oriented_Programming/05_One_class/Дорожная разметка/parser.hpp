#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include "line.hpp"

class Parser final {
private:
    std::istream& is;

public:
    Parser(std::istream& _is) : is{ _is } {};
    std::pair<std::vector<Line>, bool> getLines();
    static LineType convert(const std::string & str) {
        if (str.empty()) {
            throw std::invalid_argument("Invalid string to convert into Line!");
        }

        if (str == "continuous") {
            return continuous;
        }
        else if (str == "broken") {
            return broken;
        }
        else if (str == "changing") {
            return changing;
        }
        else if (str == "stripes") {
            return stripes;
        }
        else if (str == "empty") {
            return empty;
        }
        else {
            throw std::invalid_argument("Unknown type of line");
        }
    }
};
