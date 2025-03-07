#pragma once

#include <functional>

class SolveStrategy {
public:
    virtual ~SolveStrategy() = default;
    virtual double execute(std::function<double(double)> function, double x1, double x2, double eps, std::function<double(double)> derivative) const = 0;

protected:
    size_t max_iter = 1000000;
};

class Solver {
private:
    std::unique_ptr<SolveStrategy> strategy_;

public:
    explicit Solver(std::unique_ptr<SolveStrategy> strategy) : strategy_(std::move(strategy)) {}
    double Calculate(std::function<double(double)> function, double x1, double x2, double eps, std::function<double(double)> derivative = nullptr) const;
};
