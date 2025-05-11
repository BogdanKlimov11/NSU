#pragma once

#include <gtest/gtest.h>

#include <unordered_map>

#include "sum_all.hpp"

TEST(TestCaseName, TestName) {
    EXPECT_EQ(1, 1);
    EXPECT_TRUE(true);
}

TEST(Vector, String) {
    std::vector<std::string> v1 = {"abc", "de", "f"};
    ASSERT_EQ(sumAll(v1), "abcdef");
}

TEST(String, String) {
    std::string str1 = "abc";
    ASSERT_EQ(sumAll(str1), "abc");
}

TEST(Vector, CharToString) {
    std::vector<char> v1 = {'a', 'b', 'c'};
    ASSERT_EQ(sumAll(v1), "abc");
}

TEST(Vector, UCharToInt) {
    std::vector<unsigned char> v1 = {'a', 'b', 'c'};
    ASSERT_EQ(sumAll(v1), 294);
}

TEST(Vector, IntToLong) {
    std::vector<int> v1 = {1, 2, 3};
    ASSERT_EQ(sumAll(v1), 6);
}

TEST(Vector, DoubleToDouble) {
    std::vector<double> v1 = {1.0, 2.0, 3.0};
    ASSERT_EQ(sumAll(v1), 6.0);
}

TEST(Vector, ShortToInt) {
    std::vector<short> v1 = {1, 2, 3};
    ASSERT_EQ(sumAll(v1), 6);
}

TEST(Vector, LongToLongLong) {
    std::vector<long> v1 = {1, 2, 3};
    ASSERT_EQ(sumAll(v1), 6);
}

TEST(Vector, FloatToDouble) {
    std::vector<float> v1 = {1.0, 2.0, 3.0};
    ASSERT_EQ(sumAll(v1), 6.0);
}

TEST(Vector, LDoubleToLDouble) {
    std::vector<long double> v1 = {1.0, 2.0, 3.0};
    ASSERT_EQ(sumAll(v1), 6.0);
}

TEST(Vector, UnShortToUInt) {
    std::vector<unsigned short> v1 = {1, 2, 3};
    ASSERT_EQ(sumAll(v1), 6);
}

TEST(Vector, UInttoULong) {
    std::vector<unsigned int> v1 = {1, 2, 3};
    ASSERT_EQ(sumAll(v1), 6);
}

TEST(Vector, ULongToULong) {
    std::vector<unsigned long> v1 = {1, 2, 3};
    ASSERT_EQ(sumAll(v1), 6);
}

TEST(Map, DoubleToDouble) {
    std::map<int, double> v1 = {{1, 1.0}, {2, 2.0}, {3, 3.0}};
    ASSERT_EQ(sumAll(v1), 6.0);
}

TEST(Map, CharToString) {
    std::map<int, char> v1 = {{1, 'a'}, {2, 'b'}, {3, 'c'}};
    ASSERT_EQ(sumAll(v1), "abc");
}

TEST(Map, CharToInt) {
    std::map<int, unsigned char> v1 = {{1, 'a'}, {2, 'b'}, {3, 'c'}};
    ASSERT_EQ(sumAll(v1), 294);
}

TEST(List, Double) {
    std::list<double> l1 = {1.0, 2.0, 3.0};
    ASSERT_EQ(sumAll(l1), 6.0);
}

TEST(List, CharToString) {
    std::forward_list<char> v1 = {'a', 'b', 'c'};
    ASSERT_EQ(sumAll(v1), "abc");
}

TEST(Set, Double) {
    std::set<double> s1 = {1.0, 2.0, 3.0};
    ASSERT_EQ(sumAll(s1), 6.0);
}

TEST(Map, StringToString) {
    std::map<int, std::string> v1 = {{1, "ab"}, {2, "cd"}, {3, "ef"}};
    ASSERT_EQ(sumAll(v1), "abcdef");
}

TEST(UnorderedMap, StringToString) {
    std::unordered_map<int, std::string> v1 = {{1, "ab"}, {2, "cd"}, {3, "ef"}};
    ASSERT_EQ(sumAll(v1), "abcdef");
}

TEST(Vector, OverflowInt) {
    std::vector<int> v = {std::numeric_limits<int>::max(), std::numeric_limits<int>::max()};
    long sum = static_cast<int>(std::numeric_limits<int>::max()) * 2ll;
    auto output = sumAll(v);
    EXPECT_EQ(sumAll(v), sum);
}

TEST(Vector, OverflowShort) {
    std::vector<short> v = {std::numeric_limits<short>::max(), std::numeric_limits<short>::max()};
    int sum = static_cast<int>(std::numeric_limits<short>::max()) * 2ll;
    auto output = sumAll(v);
    EXPECT_EQ(sumAll(v), sum);
}
