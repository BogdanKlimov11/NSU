#include "secant.hpp"

double Secant::execute(std::function<double(double)> function, double x1, double x2, double eps, std::function<double(double)> derivative) const {
    double x1Initial = x1;
    double x2Initial = x2;

    double fx1 = function(x1);
    double fx2 = function(x2);

    if (std::abs(fx1) < eps) { return x1; }
    if (std::abs(fx2) < eps) { return x2; }

    for (size_t i = 0; i < max_iter; i++) {
        x1 = x2 - (x2 - x1) * fx2 / (fx2 - fx1);
        fx1 = function(x1);
        x2 = x1 - (x1 - x2) * fx1 / (fx1 - fx2);
        fx2 = function(x2);

        if (std::isnan(x1) || std::isnan(x2)) {
            return nan("");
        }

        if (std::abs(x2 - x1) < eps) {
            if (x2 < x1Initial || x2 > x2Initial) {
                return nan("");
            }
            return x2;
        }
    }
    return nan("");
}
