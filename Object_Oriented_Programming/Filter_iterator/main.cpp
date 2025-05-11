#include <vector>
#include <iostream>
#include <functional>

#include "filter_iterator.hpp"

class IsGreaterTen final {
private:
    int ten = 10;

public:
    bool operator()(int num) {
        return ten < num;
    }
    bool operator==(const IsGreaterTen& pred) const {
        return ten == pred.ten;
    }
};

int main() {
    std::function<bool(int)> is_less_3 = [](int x) { return x < 3; };
    auto is_g_10 = IsGreaterTen();
    std::vector<int> v1{1, 2, 33, 4, 2, 7, 1, 123};
    filter_iterator<IsGreaterTen, std::vector<int>::iterator> f1(is_g_10, v1.begin(), v1.end());
    filter_iterator<IsGreaterTen, std::vector<int>::iterator> f2(is_g_10, v1.begin(), v1.end());

    for (size_t i = 0; i < 4; i++) {
        try {
            std::cout << *f1 << " " << std::endl;
        }
        catch (std::out_of_range& ex) {
            std::cout << ex.what() << std::endl;
            break;
        }
        ++f1;
    }

    return 0;
}
