#pragma once

#include <random>
#include <unordered_map>
#include <unordered_set>
#include <sstream>

#include "cpp_decorator.hpp"

class CodeReviewDecorator : public CPPDecorator {
private:
    static const char* forbidden;

public:
    CodeReviewDecorator() = default;
    CodeReviewDecorator(std::shared_ptr<AbstractDecorator> component) : CPPDecorator(component) {}
    virtual std::string write(const std::string& str) override;
    static std::map<std::string, Color> BuildColorMap(const std::string& str, const std::map<std::string, Color>& special_words);
    virtual ~CodeReviewDecorator() = default;
    static std::vector<std::string> filterPunctuatuion(const std::string& str);
};
