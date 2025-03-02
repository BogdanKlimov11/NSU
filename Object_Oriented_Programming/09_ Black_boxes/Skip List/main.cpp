#include <iostream>

#include "skip_list.hpp"

int main() {
    SkipList<const char, int> test_list;
    std::pair<const char, int> val{ 'A', 100 };
    auto p = test_list.insert(val);

    try {
        std::cout << test_list.at('A') << " " << std::endl;

        auto it = test_list.begin();
        test_list.erase(it++);
    }
    catch (std::out_of_range& ex) {
        std::cout << ex.what() << std::endl;
    }

    return 0;
}
