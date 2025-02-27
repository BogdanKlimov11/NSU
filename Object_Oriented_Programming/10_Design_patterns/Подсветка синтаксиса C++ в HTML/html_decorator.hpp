#pragma once

#include <memory>
#include <sstream>

#include "decorator.hpp"

class HTMLDecorator : public Decorator {
private:
    std::string replaceCharacters(const std::string& str);
    std::vector<std::pair<std::string, std::string>> special_symbols{
        {"&", "&"},
        {"\t", " "},
        {"'", "'"},
        {"\"", "\""},
        {"<", "<"},
        {">", ">"}
    };

public:
    HTMLDecorator(std::shared_ptr<AbstractDecorator> component) : Decorator(component) {}
    std::string write(const std::string& str) override;
};
