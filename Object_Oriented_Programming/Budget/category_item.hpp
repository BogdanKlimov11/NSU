#pragma once

#include <string>
#include <vector>

struct CategoryItem {
    std::string category;
    std::string subcategory;

    bool operator==(const CategoryItem& other) const {
        return (category == other.category) && (subcategory == other.subcategory);
    }
    bool operator!=(const CategoryItem& other) const {
        return (category != other.category) || ((category == other.category) && (subcategory != other.subcategory));
    }
    bool operator<(const CategoryItem& other) const {
        return (category < other.category);
    }
};

struct Categories {
    std::vector<CategoryItem> categoriesList;
    double expectedValue;

    bool operator==(const Categories& other) const {
        return (categoriesList == other.categoriesList) && (expectedValue == other.expectedValue);
    }
    bool operator<(const Categories& other) const {
        return (categoriesList < other.categoriesList);
    }
};
