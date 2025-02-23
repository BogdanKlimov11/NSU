#pragma once

#include <vector>
#include <iostream>
#include <sstream>
#include <string>

#include "point.hpp"

class Parser {
    private:
        std::istream &is;
    
    public:
        Parser(std::istream& _is) : is(_is) {};
        std::pair<Point, bool> getNextPoint() const;
};
