#include <gtest/gtest.h>

#include <cmath>

#include "bisection.hpp"
#include "secant.hpp"
#include "newton.hpp"
#include "solver.hpp"

double log1(double x) {
    return std::log(x);
}

double dlog1(double x) {
    return 1 / x;
}

double sq1(double x) {
    return x * x - 4;
}

double dsq1(double x) {
    return 2 * x;
}

double exp1(double x) {
    return std::exp(x) - 1;
}

double dexp1(double x) {
    return std::exp(x);
}

double sq2(double x) {
    return 2 * x * x + 6 * x - 15;
}

double dsq2(double x) {
    return 4 * x + 6;
}

double coub2(double x) {
    return x * x * x + 3 * x * x + 12 * x - 8;
}

double dcoub2(double x) {
    return 3 * x * x + 6 * x + 12;
}

double coubCycle(double x) {
    return x * x * x - 2 * x - 2;
}

double dcoubCycle(double x) {
    return 3 * x * x - 2;
}

TEST(Bisection, Valid) {
    EXPECT_LE(std::abs(1 - Solver(std::make_unique<Bisection>()).Calculate(log1, 0.2, 3, 0.0000000005)), 0.0000000005);
    EXPECT_LE(std::abs(2 - Solver(std::make_unique<Bisection>()).Calculate(sq1, 0, 3, 0.000000001)), 0.000000001);
    EXPECT_LE(std::abs(Solver(std::make_unique<Bisection>()).Calculate(exp1, -1, 3, 0.0000000001)), 0.0000000001);
    EXPECT_LE(std::abs(-4.6224989991992 - Solver(std::make_unique<Bisection>()).Calculate(sq2, -6, 0, 0.00000000001)), 0.00000000001);
    EXPECT_LE(std::abs(0.57000652648718 - Solver(std::make_unique<Bisection>()).Calculate(coub2, -2, 3, 0.00000001)), 0.00000001);
    EXPECT_LE(std::abs(1.769292354238635 - Solver(std::make_unique<Secant>()).Calculate(coubCycle, 1, 3, 0.00000000000001, dcoubCycle)), 0.00000000000001);
}

TEST(Bisection, InValid) {
    EXPECT_TRUE(std::isnan(Solver(std::make_unique<Bisection>()).Calculate(log1, 2, 3, 0.05)));
    EXPECT_TRUE(std::isnan(Solver(std::make_unique<Bisection>()).Calculate(exp1, -3, -2, 0.01)));
    EXPECT_TRUE(std::isnan(Solver(std::make_unique<Bisection>()).Calculate(coub2, -2, 0, 0.1)));
    EXPECT_TRUE(std::isnan(Solver(std::make_unique<Bisection>()).Calculate(sq2, -3, -2, 0.01)));
}

TEST(Secant, Valid) {
    EXPECT_LE(std::abs(1 - Solver(std::make_unique<Secant>()).Calculate(log1, 0.2, 3, 0.0000000005)), 0.0000000005);
    EXPECT_LE(std::abs(2 - Solver(std::make_unique<Secant>()).Calculate(sq1, 0, 3, 0.000000001)), 0.000000001);
    EXPECT_LE(std::abs(Solver(std::make_unique<Secant>()).Calculate(exp1, -1, 3, 0.0000000001)), 0.0000000001);
    EXPECT_LE(std::abs(-4.6224989991992 - Solver(std::make_unique<Secant>()).Calculate(sq2, -6, 0, 0.000000001)), 0.000000001);
    EXPECT_LE(std::abs(0.57000652648718 - Solver(std::make_unique<Secant>()).Calculate(coub2, -2, 3, 0.000000001)), 0.000000001);
    EXPECT_LE(std::abs(1.769292354238635 - Solver(std::make_unique<Secant>()).Calculate(coubCycle, 1, 3, 0.00000000000001, dcoubCycle)), 0.00000000000001);
}

TEST(Secant, InValid) {
    EXPECT_TRUE(std::isnan(Solver(std::make_unique<Secant>()).Calculate(sq1, -21, 21, 0.05)));
    EXPECT_TRUE(std::isnan(Solver(std::make_unique<Secant>()).Calculate(log1, 2, 3, 0.05)));
}

TEST(Newton, Valid) {
    EXPECT_LE(std::abs(1 - Solver(std::make_unique<Newton>()).Calculate(log1, 0.2, 1.3, 0.00000005, dlog1)), 0.0000005);
    EXPECT_LE(std::abs(2 - Solver(std::make_unique<Newton>()).Calculate(sq1, 1, 4, 0.000001, dsq1)), 0.000001);
    EXPECT_LE(std::abs(0 - Solver(std::make_unique<Newton>()).Calculate(exp1, -2, 1, 0.0000001, dexp1)), 0.0000001);
    EXPECT_LE(std::abs(-4.6224989991992 - Solver(std::make_unique<Newton>()).Calculate(sq2, -6, -4, 0.0000001, dsq2)), 0.0000001);
    EXPECT_LE(std::abs(0.57000652648718 - Solver(std::make_unique<Newton>()).Calculate(coub2, 0, 3, 0.0000001, dcoub2)), 0.0000001);
    EXPECT_LE(std::abs(1.769292354238635 - Solver(std::make_unique<Newton>()).Calculate(coubCycle, 1, 3, 0.00000000000001, dcoubCycle)), 0.00000000000001);
}

TEST(Newton, InValid) {
    EXPECT_TRUE(std::isnan(Solver(std::make_unique<Newton>()).Calculate(coubCycle, -3, 3, 0.000000000001, dcoubCycle)));
    EXPECT_TRUE(std::isnan(Solver(std::make_unique<Newton>()).Calculate(log1, 4, 8, 0.00000000005, dlog1)));
    EXPECT_TRUE(std::isnan(Solver(std::make_unique<Newton>()).Calculate(exp1, -200, 0, 0.000000000001, dexp1)));
    EXPECT_TRUE(std::isnan(Solver(std::make_unique<Newton>()).Calculate(sq1, -1, 1, 0.0000000000001, dsq1)));
}
