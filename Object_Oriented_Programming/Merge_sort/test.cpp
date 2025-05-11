#pragma once

#include "gtest/gtest.h"

#include <list>
#include <deque>
#include <string>
#include <array>
#include <forward_list>

#include "merge_sort.hpp"
#include "point.hpp"

using namespace std;
using namespace merge;

template<typename T>
bool compar(const T& a, const T& b) { return (a < b); }

template<typename T>
bool pointComparator(const Point<T>& x, const Point<T>& y) { return x < y; }

TEST(Vector, Double) {
    std::vector<double> v1 = {1.0, 2.0, 7.0, 3.0, 4.0};
    std::vector<double> v2 = {1.0, 2.0, 3.0, 4.0, 7.0};
    merge::sort(v1.begin(), v1.end(), compar<double>);
    EXPECT_EQ(v1, v2);
}

TEST(Vector, Char) {
    std::vector<char> v1 = {'a', 'c', 'd', 'b'};
    std::vector<char> v2 = {'a', 'b', 'c', 'd'};
    merge::sort(v1.begin(), v1.end(), compar<char>);
    EXPECT_EQ(v1, v2);
}

TEST(Vector, Int) {
    std::vector<int> v1 = {1, 2, 7, 3, 4};
    std::vector<int> v2 = {1, 2, 3, 4, 7};
    merge::sort(v1.begin(), v1.end(), compar<int>);
    EXPECT_EQ(v1, v2);
}

TEST(Deque, Double) {
    std::deque<double> v1 = {1.0, 2.0, 7.0, 3.0, 4.0};
    merge::sort(v1.begin(), v1.end(), std::greater<double>());
    EXPECT_TRUE(std::is_sorted(std::begin(v1), std::end(v1), std::greater<double>()));
}

TEST(Deque, Char) {
    std::deque<char> v1 = {'a', 'c', 'd', 'b'};
    std::deque<char> v2 = {'a', 'b', 'c', 'd'};
    merge::sort(v1.begin(), v1.end(), compar<char>);
    EXPECT_EQ(v1, v2);
}

TEST(StdArray, Float) {
    std::array<float, 5> v = {1.0, 2.0, 7.0, 3.0, 4.0};
    merge::sort(v.begin(), v.end(), compar<float>);
    EXPECT_TRUE(std::is_sorted(v.begin(), v.end(), compar<float>));
}

TEST(StdArray, Char) {
    std::array<char, 5> v = {'a', 'b', 'c', 'd'};
    merge::sort(v.begin(), v.end(), std::greater<char>());
    EXPECT_TRUE(std::is_sorted(v.begin(), v.end(), std::greater<char>()));
}

TEST(StdArray, PointChar) {
    Point<char> p1('a');
    Point<char> p2('d');
    Point<char> p3('b');
    std::array<Point<char>, 3> v = {p1, p2, p3};
    merge::sort(std::begin(v), std::end(v), std::greater<Point<char>>());
    EXPECT_TRUE(std::is_sorted(std::begin(v), std::end(v), std::greater<Point<char>>()));
}

TEST(Array, Long) {
    long int v[] = {1, 2, 7, 3, 4};
    merge::sort(std::begin(v), std::end(v));
    EXPECT_TRUE(std::is_sorted(std::begin(v), std::end(v)));
}

TEST(Array, PointChar) {
    Point<char> p1('a');
    Point<char> p2('d');
    Point<char> p3('b');
    Point<char> v[] = {p1, p2, p3};
    merge::sort(std::begin(v), std::end(v));
    EXPECT_TRUE(std::is_sorted(std::begin(v), std::end(v)));
}

TEST(Vector, Point) {
    Point<int> p1(10);
    Point<int> p2(7);
    Point<int> p3(11);
    std::vector<Point<int>> v = {p1, p2, p3};
    merge::sort(v.begin(), v.end(), std::greater<Point<int>>());
    EXPECT_TRUE(std::is_sorted(v.begin(), v.end(), std::greater<Point<int>>()));
}

TEST(String, String) {
    std::string word = "based";
    merge::sort(word.begin(), word.end());
    EXPECT_TRUE(std::is_sorted(word.begin(), word.end()));
}
