#include <vector>

#include "date.hpp"
#include "define_day.hpp"

namespace {
    std::vector<std::string> DaysNames {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
    std::vector<std::string> MonthsNames {"January", "February", "March", "April", "May", "June",
                                          "July", "August", "September", "October", "November", "December"};
}

int Date::getDay() { return day; }
int Date::getMonth() { return month; }
int Date::getYear() { return year; }

Date::Date(int day, int month, int year) {
    if (day > 31 || month > 12 || year < 0 || day < 0 || month < 0) {
        return;
    }
    this->day = day;
    this->month = month;
    this->year = year;
}

bool Date::operator==(const Date& that) {
    return (day == that.day && month == that.month && year == that.year);
}

std::string Date::getDayAsString() {
    if (*this == Date(-1, -1, -1)) {
        return "";
    }
    return DaysNames[defineDay(day, month, year)];
}

std::string Date::getMonthAsString() {
    if (*this == Date(-1, -1, -1)) {
        return "";
    }
    return MonthsNames[month - 1];
}
