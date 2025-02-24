#include <list>
#include <deque>
#include <string>
#include <array>
#include <forward_list>

#include "gtest/gtest.h"
#include "quick_sort_concepts.hpp"
#include "point.hpp"
#include "fill_random.hpp"

template <typename T>
bool compar_less(const T& a, const T& b) { return (a < b); }

template <typename T>
bool compar_greater(const T& a, const T& b) { return (a > b); }

using namespace std;

TEST(Vector, Double) {
    std::vector<double> v1 = { 1.0, 2.0, 7.0, 3.0, 4.0 };
    std::vector<double> v2 = { 1.0, 2.0, 3.0, 4.0, 7.0 };
    quick_sort::sort(v1.begin(), v1.end(), compar_less<double>);
    EXPECT_EQ(v1, v2);
}

TEST(Vector, Char) {
    std::vector<char> v1 = { 'a', 'c', 'd', 'b' };
    std::vector<char> v2 = { 'a', 'b', 'c', 'd' };
    quick_sort::sort(v1.begin(), v1.end());
    EXPECT_EQ(v1, v2);
    EXPECT_TRUE(std::is_sorted(v1.begin(), v1.end()));
}

TEST(Vector, Int) {
    std::vector<int> v1 = { 1, 2, 7, 3, 4 };
    std::vector<int> v2 = { 1, 2, 3, 4, 7 };
    quick_sort::sort(v1.begin(), v1.end());
    EXPECT_EQ(v1, v2);
}

TEST(Deque, Int) {
    std::deque<int> v1 = { 1, 2, 7, 3, 4 };
    std::deque<int> v2 = { 1, 2, 3, 4, 7 };
    quick_sort::sort(v1.begin(), v1.end(), compar_less<int>);
    EXPECT_EQ(v1, v2);
}

TEST(Deque, Double) {
    std::deque<double> v1 = { 1.0, 3.0, 4.0, 2.0 };
    std::deque<double> v2 = { 1.0, 2.0, 3.0, 4.0 };
    quick_sort::sort(v1.begin(), v1.end(), compar_less<double>);
    EXPECT_EQ(v1, v2);
}

TEST(Deque, Char) {
    std::deque<char> v1 = { 'a', 'c', 'd', 'b' };
    std::deque<char> v2 = { 'a', 'b', 'c', 'd' };
    quick_sort::sort(v1.begin(), v1.end());
    EXPECT_EQ(v1, v2);
}

TEST(StdArray, FloatLess) {
    std::array<float, 5> v = { 1.0, 2.0, 7.0, 3.0, 4.0 };
    quick_sort::sort(v.begin(), v.end(), compar_less<float>);
    EXPECT_TRUE(std::is_sorted(v.begin(), v.end(), compar_less<float>));
}

TEST(StdArray, FloatGreater) {
    std::array<float, 5> v = { 1.0, 2.0, 7.0, 3.0, 4.0 };
    quick_sort::sort(v.begin(), v.end(), compar_greater<float>);
    EXPECT_TRUE(std::is_sorted(v.begin(), v.end(), compar_greater<float>));
}

TEST(StdArray, Char) {
    std::array<char, 5> v = { 'a', 'b', 'c', 'd' };
    quick_sort::sort(v.begin(), v.end(), std::greater<char>());
    EXPECT_TRUE(std::is_sorted(v.begin(), v.end(), std::greater<char>()));
}

TEST(Array, Long) {
    long int v[] = { 1, 2, 7, 3, 4 };
    quick_sort::sort(std::begin(v), std::end(v));
    EXPECT_TRUE(std::is_sorted(std::begin(v), std::end(v)));
}

TEST(String, String) {
    std::string word = "based";
    quick_sort::sort(word.begin(), word.end());
    EXPECT_TRUE(std::is_sorted(word.begin(), word.end()));
}

TEST(Vector, Point) {
    Point<int> p1(10);
    Point<int> p2(7);
    Point<int> p3(11);
    std::vector<Point<int>> v = { p1, p2, p3 };
    quick_sort::sort(v.begin(), v.end(), std::greater<Point<int>>());
    EXPECT_TRUE(std::is_sorted(v.begin(), v.end(), std::greater<Point<int>>()));
}

TEST(StdArray, Point) {
    Point<int> p1(10);
    Point<int> p2(7);
    Point<int> p3(11);
    std::array<Point<int>, 3> v = { p1, p2, p3 };
    quick_sort::sort(v.begin(), v.end(), std::greater<Point<int>>());
    EXPECT_TRUE(std::is_sorted(v.begin(), v.end(), std::greater<Point<int>>()));
}

TEST(Deque, Point) {
    Point<int> p1(10);
    Point<int> p2(7);
    Point<int> p3(11);
    std::deque<Point<int>> v = { p1, p2, p3 };
    quick_sort::sort(v.begin(), v.end(), std::greater<Point<int>>());
    EXPECT_TRUE(std::is_sorted(v.begin(), v.end(), std::greater<Point<int>>()));
}

TEST(FillRandom, Int) {
    std::deque<int> v1(10, 0);
    std::deque<int> v2 = v1;
    fillRand(v1.begin(), v1.end(), 2, 100);
    EXPECT_FALSE(std::equal(v1.begin(), v1.end(), v2.begin(), v2.end()));
}

TEST(FillRandom, Double) {
    std::vector<double> v1(10, 0.0);
    std::vector<double> v2 = v1;
    fillRand(v1.begin(), v1.end(), 2.0, 100.0);
    EXPECT_FALSE(std::equal(v1.begin(), v1.end(), v2.begin(), v2.end()));
}

TEST(StdArray, RandomDouble) {
    std::array<double, 10> v1;
    fillRand(v1.begin(), v1.end(), 1.0, 50.0);
    quick_sort::sort(v1.begin(), v1.end());
    EXPECT_TRUE(std::is_sorted(v1.begin(), v1.end()));
}

TEST(Vector, RandomDouble) {
    std::vector<double> v1(10);
    fillRand(v1.begin(), v1.end(), 1.0, 50.0);
    quick_sort::sort(v1.begin(), v1.end(), compar_less<double>);
    EXPECT_TRUE(std::is_sorted(v1.begin(), v1.end()));
}

TEST(Array, RandomLong) {
    long int v1[100];
    fillRand(std::begin(v1), std::end(v1), std::numeric_limits<long>::lowest(), std::numeric_limits<long>::max());
    quick_sort::sort(std::begin(v1), std::begin(v1), compar_less<long>);
    EXPECT_TRUE(std::is_sorted(std::begin(v1), std::begin(v1)));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
