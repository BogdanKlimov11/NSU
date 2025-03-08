#pragma once

#include <utility>
#include <exception>
#include <iostream>

#include "bitmap.hpp"

class MedianFilterExceptions : std::exception {
protected:
    virtual void Message() = 0;
};

class WrongInputExceptions : MedianFilterExceptions {
public:
    inline void Message() override {
        std::cerr << "Wrong input data! Check your input." << std::endl;
    }
};

class WrongFiltrationParamsExceptions: MedianFilterExceptions {
public:
    inline void Message() override {
        std::cerr << "Wrong filtration params! Check your neighbour size and filtration threshold." << std::endl;
    }
};
