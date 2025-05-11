#pragma once

#include <string>
#include <vector>

class Date final {
public:
    explicit Date(int day, int month, int year);
    Date() = default;
    int getYear();
    int getMonth();
    int getDay();
    std::string getDayAsString();
    std::string getMonthAsString();
    bool operator==(const Date& that);

private:
    int year;
    int month;
    int day;
};
