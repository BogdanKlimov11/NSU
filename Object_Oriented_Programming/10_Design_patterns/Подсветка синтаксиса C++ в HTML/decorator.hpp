#pragma once

#include <vector>
#include <memory>

#include "abstract_decorator.hpp"

class Decorator : public AbstractDecorator {
protected:
    std::shared_ptr<AbstractDecorator> component_;

public:
    Decorator(std::shared_ptr<AbstractDecorator> component) : component_(component) {}
    std::string Decorate(const std::string& str) {
        if (component_)
            return component_->write(str);
        return str;
    }
};
