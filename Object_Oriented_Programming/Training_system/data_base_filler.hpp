#pragma once

#include <sstream>

#include "factory.hpp"

class DataBaseFiller {
private:
    std::istream& is;
    Factory& factory;

public:
    DataBaseFiller(std::istream& _is, Factory& f) : is(_is), factory(f) {}
    std::shared_ptr<Item> getNextItem();
};
