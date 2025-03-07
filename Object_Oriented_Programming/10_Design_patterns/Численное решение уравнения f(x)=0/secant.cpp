#include "secant.hpp"

double Secant::execute(std::function<double(double)> function, double x1, double x2, double eps, std::function<double(double)> derivative) const {
    double x1_ = x1;
    double x2_ = x2;

    double Fx1 = function(x1);
    double Fx2 = function(x2);

    if (abs(Fx1) < eps) { return x1; }
    if (abs(Fx2) < eps) { return x2; }

    for (size_t i = 0; i < max_iter; i++) {
        x1 = x2 - (x2 - x1) * Fx2 / (Fx2 - Fx1);
        Fx1 = function(x1);
        x2 = x1 - (x1 - x2) * Fx1 / (Fx1 - Fx2);
        Fx2 = function(x2);

        if (std::isnan(x1) || std::isnan(x2)) {
            return nan("");
        }

        if (std::abs(x2 - x1) < eps) {
            if (x2 < x1_ || x2 > x2_) {
                return nan("");
            }
            return x2;
        }
    }
    return nan("");
}
