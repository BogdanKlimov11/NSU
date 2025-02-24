#pragma once

#include <iostream>
#include <string>
#include <sstream>

class Parser final {
private:
    std::istream& is;

public:
    Parser(std::istream& _is) : is{ _is } {};
    std::pair<std::pair<std::string, std::string>, bool> getNextFriendship();
};
