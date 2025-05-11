#include "writer.hpp"

void Writer::writeSpendings(const std::vector<itemInfo>& spendings) {
    for (const auto& category : spendings) {
        for (const auto& item : category.categoriesList) {
            os << item.category;
            if (!item.subcategory.empty()) {
                os << ':' << item.subcategory;
            }
            if (item != category.categoriesList.back()) {
                os << '+';
            }
        }
        os << '    ' << category.expected_price << '    ' << category.real_price << '    ';
        os << category.percent << '%' << std::endl;
    }
}
