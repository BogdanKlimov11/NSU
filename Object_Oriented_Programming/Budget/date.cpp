#include "date.hpp"

bool Date::isDateCorrect(size_t day, size_t month, size_t year) {
    if (day == 0 || day > 31 || month == 0 || month > 12) {
        return false;
    }
    if (month == 4 || month == 6 || month == 9 || month == 11) {
        return day <= 30;
    }
    if (month == 2) {
        if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
            return day <= 29;
        }
        return day <= 28;
    }
    return true;
}

Date::Date(const std::string& date) {
    std::stringstream date_stream{date};
    size_t _day, _month, _year;
    char sep;
    date_stream >> _day >> sep >> _month >> sep >> _year;
    if (!isDateCorrect(_day, _month, _year)) {
        throw std::invalid_argument("Incorrect date!");
    }
    day = _day;
    month = _month;
    year = _year;
}

bool Date::operator<=(const Date& other) const {
    size_t this_date = year * 365 + month * 31 + day;
    size_t other_date = other.year * 365 + other.month * 31 + other.day;
    return this_date <= other_date;
}

bool Date::operator<(const Date& other) const {
    size_t this_date = year * 365 + month * 31 + day;
    size_t other_date = other.year * 365 + other.month * 31 + other.day;
    return this_date < other_date;
}

bool Date::operator==(const Date& other) const {
    size_t this_date = year * 365 + month * 31 + day;
    size_t other_date = other.year * 365 + other.month * 31 + other.day;
    return this_date == other_date;
}
