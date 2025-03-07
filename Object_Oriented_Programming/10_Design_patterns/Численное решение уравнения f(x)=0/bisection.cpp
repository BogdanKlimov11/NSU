#include "bisection.hpp"

double Bisection::execute(std::function<double(double)> function, double x1, double x2, double eps, std::function<double(double)> derivative) const {
    double Fx1 = function(x1);
    double Fx2 = function(x2);

    if (abs(Fx1) < eps) { return x1; }
    if (abs(Fx2) < eps) { return x2; }

    double xi;
    double Fxi;

    for (size_t i = 0; i < max_iter; i++) {
        double dx = (x2 - x1) / 2;
        xi = x1 + dx;

        Fxi = function(xi);

        if (std::signbit(Fx1) != std::signbit(Fxi)) {
            x2 = xi;
        } else {
            x1 = xi;
        }

        if (std::isnan(x1) || std::isnan(x2)) {
            return nan("");
        }

        if (std::abs(x2 - x1) < eps) {
            if (std::abs(Fxi) < std::abs(function(xi + 2 * eps)) && std::abs(Fxi) < std::abs(function(xi - 2 * eps))) {
                return xi;
            }
            return nan("");
        }
    }

    return nan("");
}
