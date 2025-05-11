#include "budget.hpp"

void Budget::addExpectedSpendings(Categories& categories) {
    for (const auto& item : categories.categoriesList) {
        auto node = std::make_pair(item, categories);
        ExpectedSpendings.insert(node);
    }
}

void Budget::checkOtherSpending(std::vector<itemInfo>& result, double other_spendings) {
    CategoryItem other{"other"};
    std::vector<CategoryItem> otherlist{other};
    auto other_iter = std::find_if(result.begin(), result.end(),
        [&otherlist](itemInfo& item) { return item.categoriesList == otherlist; });
    if (other_iter == result.end()) {
        result.push_back(itemInfo{otherlist, 0.0, 0.0, 0.0});
        result.back().real_price = other_spendings;
    } else {
        other_iter->real_price += other_spendings;
        other_iter->percent = other_iter->real_price * 100 / other_iter->expected_price;
    }
}

std::vector<itemInfo> Budget::initResult() {
    std::vector<itemInfo> result;
    for (const auto& it : ExpectedSpendings) {
        result.push_back(itemInfo{it.second.categoriesList, it.second.expectedValue, 0.0, 0.0});
    }
    std::sort(result.begin(), result.end());
    auto tmp = std::unique(result.begin(), result.end());
    result.erase(tmp, result.end());
    return result;
}

void Budget::expectedSpendingsFiller(ExpectedSpendingParser& parser) {
    while (true) {
        auto item = parser.getNextItem();
        if (!item.second)
            break;
        addExpectedSpendings(item.first);
    }
}

void Budget::realSpendingsFiller(RealSpendingParser& parser) {
    while (true) {
        auto item = parser.getNextItem();
        if (!item.second)
            break;
        RealSpendings.push_back(item.first);
    }
    std::sort(RealSpendings.begin(), RealSpendings.end(), std::less<Spending>());
}

std::vector<itemInfo> Budget::calculate(const Date& start_date, const Date& end_date) {
    if (RealSpendings.empty() || ExpectedSpendings.empty()) {
        throw std::invalid_argument("Budget data is empty!");
    }
    if (start_date == Date() || end_date == Date()) {
        throw std::invalid_argument("Date is incorrect!");
    }
    auto result = initResult();
    double other_spendings = 0.0;

    auto real_spending = std::find_if(RealSpendings.begin(), RealSpendings.end(),
        [&start_date](Spending& spend) { return start_date <= spend.getDate(); });
    for (; real_spending < RealSpendings.end(); real_spending++) {
        if (end_date < real_spending->getDate()) {
            break;
        }
        auto key_it = ExpectedSpendings.find(real_spending->getItem());
        if (key_it == ExpectedSpendings.end()) {
            other_spendings += real_spending->getValue();
        } else {
            const auto& categories_list = key_it->second.categoriesList;
            auto result_iterator = std::find_if(result.begin(), result.end(),
                [&categories_list](itemInfo& item) { return item.categoriesList == categories_list; });
            result_iterator->real_price += real_spending->getValue();
            if (result_iterator->expected_price != 0) {
                result_iterator->percent = result_iterator->real_price * 100 / result_iterator->expected_price;
            }
        }
    }
    checkOtherSpending(result, other_spendings);
    return result;
}
