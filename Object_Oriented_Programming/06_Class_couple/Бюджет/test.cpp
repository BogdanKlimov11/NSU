#include "gtest/gtest.h"

#include <list>
#include <deque>
#include <string>
#include <array>
#include <forward_list>

#include "quick_sort.hpp"
#include "point.hpp"
#include "fill_random.hpp"

template<typename T>
bool compar_less(const T& a, const T& b) { return a < b; }

template<typename T>
bool compar_greater(const T& a, const T& b) { return a > b; }

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
    array<char, 5> v = {'a', 'b', 'c', 'd'};
    quick_sort::sort(v.begin(), v.end(), greater<char>());
    EXPECT_TRUE(is_sorted(v.begin(), v.end(), greater<char>()));
}

TEST(Array, Long) {
    long v[] = {1, 2, 7, 3, 4};
    quick_sort::sort(begin(v), end(v));
    EXPECT_TRUE(is_sorted(begin(v), end(v)));
}

TEST(String, String) {
    string word = "based";
    quick_sort::sort(word.begin(), word.end());
    EXPECT_TRUE(is_sorted(word.begin(), word.end()));
}

TEST(Vector, Point) {
    vector<Point<int>> v = {Point<int>(10), Point<int>(7), Point<int>(11)};
    quick_sort::sort(v.begin(), v.end(), greater<Point<int>>());
    EXPECT_TRUE(is_sorted(v.begin(), v.end(), greater<Point<int>>()));
}

TEST(StdArray, Point) {
    array<Point<int>, 3> v = {Point<int>(10), Point<int>(7), Point<int>(11)};
    quick_sort::sort(v.begin(), v.end(), greater<Point<int>>());
    EXPECT_TRUE(is_sorted(v.begin(), v.end(), greater<Point<int>>()));
}

TEST(Deque, Point) {
    deque<Point<int>> v = {Point<int>(10), Point<int>(7), Point<int>(11)};
    quick_sort::sort(v.begin(), v.end(), greater<Point<int>>());
    EXPECT_TRUE(is_sorted(v.begin(), v.end(), greater<Point<int>>()));
}

TEST(FillRandom, Int) {
    deque<int> v1(10, 0), v2 = v1;
    fillRand(v1.begin(), v1.end(), 2, 100);
    EXPECT_FALSE(equal(v1.begin(), v1.end(), v2.begin(), v2.end()));
}

TEST(FillRandom, Double) {
    vector<double> v1(10, 0.0), v2 = v1;
    fillRand(v1.begin(), v1.end(), 2.0, 100.0);
    EXPECT_FALSE(equal(v1.begin(), v1.end(), v2.begin(), v2.end()));
}

TEST(StdArray, RandomDouble) {
    array<double, 10> v1;
    fillRand(v1.begin(), v1.end(), 1.0, 50.0);
    quick_sort::sort(v1.begin(), v1.end());
    EXPECT_TRUE(is_sorted(v1.begin(), v1.end()));
}

TEST(Vector, RandomDouble) {
    vector<double> v1(10);
    fillRand(v1.begin(), v1.end(), 1.0, 50.0);
    quick_sort::sort(v1.begin(), v1.end(), compar_less<double>);
    EXPECT_TRUE(is_sorted(v1.begin(), v1.end()));
}

TEST(Array, RandomLong) {
    long v1[100];
    fillRand(begin(v1), end(v1), numeric_limits<long>::lowest(), numeric_limits<long>::max());
    quick_sort::sort(begin(v1), begin(v1), compar_less<long>);
    EXPECT_TRUE(is_sorted(begin(v1), begin(v1)));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
