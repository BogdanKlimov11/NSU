#pragma once

#include <string>
#include <iostream>
#include <vector>

#include "date.hpp"

enum class RangeType {
    RANGE,
    YEAR,
    MONTH,
    UNDEFINED
};

struct Range final {
    RangeType type;
    Date dateBegin;
    Date dateEnd;
    bool orient;
    bool yearEveryMonth = false;
    bool yearOnce = false;
};

Range CreateRange(std::vector<std::string> source);
std::vector<std::string> streamReader(std::istream& input);
