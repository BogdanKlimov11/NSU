#pragma once

#include <sstream>
#include <map>
#include <iostream>

#include "decorator.hpp"
#include "color.hpp"

class CPPDecorator : public Decorator {
protected:
    std::map<std::string, Color> key_words = {
        {"alignas", {0, 0, 255}},
        {"alignof", {0, 0, 255}},
        {"and", {0, 0, 255}},
        {"and_eq", {0, 0, 255}},
        {"asm", {0, 0, 255}},
        {"atomic_cancel", {0, 0, 255}},
        {"atomic_commit", {0, 0, 255}},
        {"atomic_noexcept", {0, 0, 255}},
        {"auto", {0, 0, 255}},
        {"bitand", {0, 0, 255}},
        {"bitor", {0, 0, 255}},
        {"bool", {0, 0, 255}},
        {"break", {0, 0, 255}},
        {"case", {0, 0, 255}},
        {"catch", {0, 0, 255}},
        {"char", {0, 0, 255}},
        {"char8_t", {0, 0, 255}},
        {"char16_t", {0, 0, 255}},
        {"char32_t", {0, 0, 255}},
        {"class", {0, 0, 255}},
        {"compl", {0, 0, 255}},
        {"concept", {0, 0, 255}},
        {"const", {0, 0, 255}},
        {"consteval", {0, 0, 255}},
        {"constexpr", {0, 0, 255}},
        {"constinit", {0, 0, 255}},
        {"const_cast", {0, 0, 255}},
        {"continue", {0, 0, 255}},
        {"co_await", {0, 0, 255}},
        {"co_return", {0, 0, 255}},
        {"co_yield", {0, 0, 255}},
        {"decltype", {0, 0, 255}},
        {"default", {0, 0, 255}},
        {"delete", {0, 0, 255}},
        {"do", {0, 0, 255}},
        {"double", {0, 0, 255}},
        {"dynamic_cast", {0, 0, 255}},
        {"else", {0, 0, 255}},
        {"enum", {0, 0, 255}},
        {"explicit", {0, 0, 255}},
        {"export", {0, 0, 255}},
        {"extern", {0, 0, 255}},
        {"false", {0, 0, 255}},
        {"float", {0, 0, 255}},
        {"for", {0, 0, 255}},
        {"friend", {0, 0, 255}},
        {"goto", {0, 0, 255}},
        {"if", {0, 0, 255}},
        {"inline", {0, 0, 255}},
        {"int", {0, 0, 255}},
        {"long", {0, 0, 255}},
        {"mutable", {0, 0, 255}},
        {"namespace", {0, 0, 255}},
        {"new", {0, 0, 255}},
        {"noexcept", {0, 0, 255}},
        {"not", {0, 0, 255}},
        {"not_eq", {0, 0, 255}},
        {"nullptr", {0, 0, 255}},
        {"operator", {0, 0, 255}},
        {"or", {0, 0, 255}},
        {"or_eq", {0, 0, 255}},
        {"private", {0, 0, 255}},
        {"protected", {0, 0, 255}},
        {"public", {0, 0, 255}},
        {"reflexpr", {0, 0, 255}},
        {"register", {0, 0, 255}},
        {"reinterpret_cast", {0, 0, 255}},
        {"requires", {0, 0, 255}},
        {"return", {0, 0, 255}},
        {"short", {0, 0, 255}},
        {"signed", {0, 0, 255}},
        {"sizeof", {0, 0, 255}},
        {"static", {0, 0, 255}},
        {"static_assert", {0, 0, 255}},
        {"static_cast", {0, 0, 255}},
        {"struct", {0, 0, 255}},
        {"switch", {0, 0, 255}},
        {"synchronized", {0, 0, 255}},
        {"template", {0, 0, 255}},
        {"this", {0, 0, 255}},
        {"thread_local", {0, 0, 255}},
        {"throw", {0, 0, 255}},
        {"true", {0, 0, 255}},
        {"try", {0, 0, 255}},
        {"typedef", {0, 0, 255}},
        {"typeid", {0, 0, 255}},
        {"typename", {0, 0, 255}},
        {"union", {0, 0, 255}},
        {"unsigned", {0, 0, 255}},
        {"using", {0, 0, 255}},
        {"virtual", {0, 0, 255}},
        {"void", {0, 0, 255}},
        {"volatile", {0, 0, 255}},
        {"wchar_t", {0, 0, 255}},
        {"while", {0, 0, 255}},
        {"xor", {0, 0, 255}},
        {"xor_eq", {0, 0, 255}}
    };

public:
    CPPDecorator() = default;
    CPPDecorator(std::shared_ptr<AbstractDecorator> component) : Decorator(component) {}
    virtual std::string write(const std::string& str) override;
    static std::string Colorize(const std::string& str, const std::map<std::string, Color>& words);
    virtual ~CPPDecorator() = default;
};
