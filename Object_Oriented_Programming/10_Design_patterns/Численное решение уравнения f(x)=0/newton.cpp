#include "newton.hpp"

double Newton::execute(std::function<double(double)> function, double x1, double x2, double eps, std::function<double(double)> derivative) const {
    double x = (x1 + x2) / 2;
    double fx1 = function(x);
    if (std::abs(fx1) < eps) { return x; }
    double xi = x - fx1 / derivative(x);

    for (size_t i = 0; i < max_iter; i++) {
        x = xi;
        xi = x - function(x) / derivative(x);

        if (std::isnan(xi) || std::isnan(x)) {
            return nan("");
        }

        if (std::abs(xi - x) < eps) {
            if (xi < x1 || xi > x2) {
                return nan("");
            }
            return xi;
        }
    }
    return nan("");
}
