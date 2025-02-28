#include <algorithm>
#include <iostream>

#include "quick_sort.hpp"

int main() {
    std::vector<double> v1 = {1.0, 8.0, 5.0, 3.0, 4.0, 2.0};
    
    quick_sort::sort(v1.begin(), v1.end());

    for (const auto& i : v1) {
        std::cout << i << " ";
    }
    
    return 0;
}
