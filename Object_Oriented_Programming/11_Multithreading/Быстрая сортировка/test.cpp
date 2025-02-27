#include "gtest/gtest.h"

#include <deque>
#include <array>
#include <random>

#include "quick_sort.hpp"
#include "policies.hpp"

template <typename T>
bool compar_less(const T& a, const T& b) {
    return a < b;
}

template <typename T>
bool compar_greater(const T& a, const T& b) {
    return a > b;
}

using namespace std;

TEST(Vector, Double) {
    vector<double> v1 = {1.0, 2.0, 7.0, 3.0, 4.0};
    vector<double> v2 = {1.0, 2.0, 3.0, 4.0, 7.0};
    quick_sort::sort(v1.begin(), v1.end(), compar_less<double>);
    EXPECT_EQ(v1, v2);
}

TEST(Vector, Char) {
    vector<char> v1 = {'a', 'c', 'd', 'b'};
    vector<char> v2 = {'a', 'b', 'c', 'd'};
    quick_sort::sort(v1.begin(), v1.end());
    EXPECT_EQ(v1, v2);
    EXPECT_TRUE(is_sorted(v1.begin(), v1.end()));
}

TEST(Vector, Int) {
    vector<int> v1 = {1, 2, 7, 3, 4};
    vector<int> v2 = {1, 2, 3, 4, 7};
    quick_sort::sort(v1.begin(), v1.end());
    EXPECT_EQ(v1, v2);
}

TEST(Deque, Int) {
    deque<int> v1 = {1, 2, 7, 3, 4};
    deque<int> v2 = {1, 2, 3, 4, 7};
    quick_sort::sort(v1.begin(), v1.end(), compar_less<int>);
    EXPECT_EQ(v1, v2);
}

TEST(Deque, Double) {
    deque<double> v1 = {1.0, 3.0, 4.0, 2.0};
    deque<double> v2 = {1.0, 2.0, 3.0, 4.0};
    quick_sort::sort(v1.begin(), v1.end(), compar_less<double>);
    EXPECT_EQ(v1, v2);
}

TEST(Deque, Char) {
    deque<char> v1 = {'a', 'c', 'd', 'b'};
    deque<char> v2 = {'a', 'b', 'c', 'd'};
    quick_sort::sort(v1.begin(), v1.end());
    EXPECT_EQ(v1, v2);
}

TEST(StdArray, FloatLess) {
    array<float, 5> v = {1.0, 2.0, 7.0, 3.0, 4.0};
    quick_sort::sort(v.begin(), v.end(), compar_less<float>);
    EXPECT_TRUE(is_sorted(v.begin(), v.end(), compar_less<float>));
}

TEST(StdArray, FloatGreater) {
    array<float, 5> v = {1.0, 2.0, 7.0, 3.0, 4.0};
    quick_sort::sort(v.begin(), v.end(), compar_greater<float>);
    EXPECT_TRUE(is_sorted(v.begin(), v.end(), compar_greater<float>));
}

TEST(StdArray, Char) {
    array<char, 4> v = {'a', 'b', 'c', 'd'};
    quick_sort::sort(v.begin(), v.end(), greater<char>());
    EXPECT_TRUE(is_sorted(v.begin(), v.end(), greater<char>()));
}

TEST(Policy, Auto) {
    AutoPolicy p;
    EXPECT_EQ(p.get_threads_number(1100), 1);
    EXPECT_EQ(p.get_threads_number(1101), 2);
    EXPECT_EQ(p.get_threads_number(2103), 3);
    EXPECT_EQ(p.get_threads_number(3604), 4);
    EXPECT_EQ(p.get_threads_number(11053), 5);
    EXPECT_EQ(p.get_threads_number(37000), 6);
    EXPECT_EQ(p.get_threads_number(66200), 7);
    EXPECT_EQ(p.get_threads_number(88800), 8);
    EXPECT_EQ(p.get_threads_number(137000), 9);
    EXPECT_EQ(p.get_threads_number(1400000), 10);
    EXPECT_EQ(p.get_threads_number(5000000), 11);
}

TEST(Vector, IntBig) {
    const int size = 200'000;
    random_device rnd_device;
    mt19937 mersenne_engine{rnd_device()};
    uniform_int_distribution<int> dist{0, size};
    auto gen = [&dist, &mersenne_engine]() {
        return dist(mersenne_engine);
    };
    vector<int> vec(size);
    generate(begin(vec), end(vec), gen);
    quick_sort::sort(vec.begin(), vec.end());
    EXPECT_TRUE(is_sorted(vec.begin(), vec.end()));
}
