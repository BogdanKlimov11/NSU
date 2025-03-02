#include "spending.hpp"

bool Spending::operator==(const Spending& other) const {
    return (date == other.date && item == other.item && value == other.value);
}

bool Spending::operator<(const Spending& other) const {
    return (date < other.date) || 
           ((date == other.date) && (item < other.item)) ||
           ((date == other.date) && (item == other.item) && (value < other.value));
}
