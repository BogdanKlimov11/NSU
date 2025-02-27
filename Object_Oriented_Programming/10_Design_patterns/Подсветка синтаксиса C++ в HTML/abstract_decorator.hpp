#pragma once

#include <string>

class AbstractDecorator {
public:
    virtual std::string write(const std::string& str) = 0;
    virtual ~AbstractDecorator() = default;
};
