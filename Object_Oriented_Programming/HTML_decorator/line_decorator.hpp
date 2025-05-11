#pragma once

#include <sstream>
#include <iomanip>

#include "decorator.hpp"

class LineDecorator final : public Decorator {
public:
    LineDecorator() = default;
    LineDecorator(std::shared_ptr<AbstractDecorator> component) : Decorator(component) {}
    std::string write(const std::string& str) override;
    virtual ~LineDecorator() = default;
};
