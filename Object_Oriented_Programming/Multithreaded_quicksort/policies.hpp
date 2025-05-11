#pragma once

#include <thread>
#include <map>

class AbstractPolicy {
protected:
    const size_t max_size_of_threads = std::thread::hardware_concurrency();

public:
    AbstractPolicy() = default;
    virtual size_t get_threads_number(size_t size) const = 0;
    virtual ~AbstractPolicy() = default;
};

class AutoPolicy : public AbstractPolicy {
    std::map<size_t, size_t> optimal_threads{
        {1100, 2},
        {2100, 3},
        {3600, 4},
        {11000, 5},
        {31000, 6},
        {66000, 7},
        {81000, 8},
        {136000, 9},
        {1300000, 10},
        {4000000, 11}
    };

public:
    AutoPolicy() = default;
    virtual size_t get_threads_number(size_t size_of_container) const override {
        if (size_of_container <= 1100) {
            return 1;
        }
        auto it = optimal_threads.lower_bound(size_of_container);
        return (--it)->second;
    }
    virtual ~AutoPolicy() = default;
};

class PolicyReturnedNumber : public AbstractPolicy {
    const size_t min_size = 100;
    size_t threads_number;

public:
    PolicyReturnedNumber(size_t threads_number_) : threads_number(threads_number_) {}
    virtual size_t get_threads_number(size_t size_of_container) const override {
        auto size_for_thread = size_of_container / threads_number;
        if (size_for_thread < min_size) {
            return (size_of_container / min_size) == 0 ? 1 : (size_of_container / min_size);
        }
        return threads_number;
    }
    virtual ~PolicyReturnedNumber() = default;
};

class PolicyOneThread : public AbstractPolicy {
public:
    PolicyOneThread() = default;
    virtual size_t get_threads_number(size_t size = 100) const override {
        return 1;
    }
    virtual ~PolicyOneThread() = default;
};
