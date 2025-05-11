#pragma once

#include <map>
#include <vector>
#include <stdexcept>

#include "expected_spending_parser.hpp"
#include "real_spending_parser.hpp"
#include "date.hpp"
#include "spending.hpp"
#include "category_item.hpp"

struct itemInfo {
    std::vector<CategoryItem> categoriesList;
    double expected_price;
    double real_price;
    double percent;

    bool operator==(const itemInfo& other) const {
        return (categoriesList == other.categoriesList);
    }
    bool operator<(const itemInfo& other) const {
        return (categoriesList < other.categoriesList);
    }
};

class Budget {
private:
    std::map<CategoryItem, Categories> ExpectedSpendings;
    std::vector<Spending> RealSpendings;

    void addExpectedSpendings(Categories& categories);
    void checkOtherSpending(std::vector<itemInfo>& result, double other_spendings);
    std::vector<itemInfo> initResult();

public:
    Budget() = default;
    void expectedSpendingsFiller(ExpectedSpendingParser& parser);
    void realSpendingsFiller(RealSpendingParser& parser);
    std::vector<itemInfo> calculate(const Date& start, const Date& end);
};
