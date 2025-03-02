#pragma once

#include <string>
#include <type_traits>

#include "traits.hpp"

template <typename T, std::enable_if_t<(std::is_arithmetic_v<typename T::value_type> && !std::is_same_v<T, std::string>) || std::is_same_v<typename T::value_type, std::string>, int> = 0>
auto sumAll(const T& container) {
    typedef typename Traits<typename T::value_type>::Trait Trait;
    Trait sum = Trait();
    for (const auto& it : container) {
        sum += it;
    }
    return sum;
}

template <typename T, std::enable_if_t<std::is_arithmetic_v<T> || std::is_same_v<T, std::string>, int> = 0>
auto sumAll(const T& element) {
    return element;
}

template <typename T, std::enable_if_t<std::is_arithmetic_v<typename T::mapped_type> || std::is_same_v<typename T::mapped_type, std::string>, int> = 0>
auto sumAll(const T& container) {
    typedef typename Traits<typename T::mapped_type>::Trait Trait;
    Trait sum = Trait();
    for (const auto& it : container) {
        sum += it.second;
    }
    return sum;
}
