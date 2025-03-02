#pragma once

#include <iostream>
#include <vector>
#include <sstream>
#include <string>

#include "budget.hpp"

class Writer {
private:
    std::ostream& os;

public:
    Writer(std::ostream& _os) : os(_os) {}
    void writeSpendings(const std::vector<itemInfo>& spendings);
};
