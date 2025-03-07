#include "solver.hpp"

double Solver::Calculate(std::function<double(double)> function, double x1, double x2, double eps, std::function<double(double)> derivative) const {
    double result = this->strategy_->execute(function, x1, x2, eps, derivative);
    return result;
}
