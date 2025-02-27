#include <algorithm>
#include <random>
#include <chrono>
#include <iostream>
#include <fstream>

#include "policies.hpp"
#include "quick_sort.hpp"

template<typename T>
bool compar_less(const T& a, const T& b) {
    return (a < b);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Format:\n main output.xlsx\n";
        exit(-1);
    }

    std::ofstream fout(argv[1]);
    std::cout << "size:\t" << "number of threads:\t" << "time:" << std::endl;
    fout << "size;" << "number of threads;" << "time\n";

    std::random_device rnd_device;
    std::mt19937 mersenne_engine{ rnd_device() };
    std::uniform_int_distribution<int> dist{ 0, 100 };
    auto gen = [&dist, &mersenne_engine]() {
        return dist(mersenne_engine);
    };

    size_t size = 5000000;
    auto max_threads_number = std::thread::hardware_concurrency();

    while (size < 50000000) {
        std::vector<int> vec(size);
        std::generate(begin(vec), end(vec), gen);

        for (size_t i = 1; i < max_threads_number; ++i) {
            PolicyReturnedNumber policy(i);
            std::cout << vec.size() << "\t";
            std::cout << policy.get_threads_number(vec.size()) << "\t";

            auto start = std::chrono::steady_clock::now();
            quick_sort::sort(vec.begin(), vec.end(), compar_less<int>, policy);
            auto end = std::chrono::steady_clock::now();
            auto elapsed_seconds = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << elapsed_seconds.count() << std::endl;

            fout << vec.size() << ";" << policy.get_threads_number(vec.size()) << ";" << elapsed_seconds.count() << "\n";
        }
        size = size + 1000000;
    }
    fout.close();
}
