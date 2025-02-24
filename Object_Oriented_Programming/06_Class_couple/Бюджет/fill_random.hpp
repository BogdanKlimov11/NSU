#pragma once

#include <iterator>
#include <random>
#include <algorithm>

template<typename Iterator, typename v_type = typename std::iterator_traits<Iterator>::value_type>
std::enable_if_t<std::is_integral_v<v_type>, void>
fillRand(Iterator begin, Iterator end, v_type min_, v_type max_, unsigned int seed = 0) {
    std::default_random_engine generator(seed);
    std::uniform_int_distribution<v_type> distribution(min_, max_);
    std::generate(begin, end, [&]() { return distribution(generator); });
}

template<typename Iterator, typename v_type = typename std::iterator_traits<Iterator>::value_type>
std::enable_if_t<std::is_floating_point_v<v_type>, void>
fillRand(Iterator begin, Iterator end, v_type min_, v_type max_, unsigned int seed = 0) {
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<v_type> distribution(min_, max_);
    std::generate(begin, end, [&]() { return distribution(generator); });
}
