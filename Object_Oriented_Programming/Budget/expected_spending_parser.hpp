#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <utility>

#include "category_item.hpp"

class ExpectedSpendingParser final {
private:
    std::istream& is;

public:
    ExpectedSpendingParser(std::istream& _is) : is{_is} {}
    std::pair<Categories, bool> getNextItem();
    std::vector<CategoryItem> getPath(const std::string& str);
};
