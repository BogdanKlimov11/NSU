#pragma once

#include <string>

#include "date.hpp"
#include "category_item.hpp"

class Spending {
private:
    Date date;
    CategoryItem item;
    double value;

public:
    Spending() = default;
    Spending(const Date& _date, const std::string& _category, const std::string& _subcategory, double _value) :
        date(_date), item{_category, _subcategory}, value(_value) {}
    const CategoryItem& getItem() const { return item; }
    const Date& getDate() const { return date; }
    const double getValue() const { return value; }
    bool operator==(const Spending& other) const;
    bool operator<(const Spending& other) const;
};
