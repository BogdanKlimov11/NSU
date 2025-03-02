#pragma once

#include <stdexcept>
#include <sstream>

class Date final {
private:
    size_t day;
    size_t month;
    size_t year;

    bool isDateCorrect(size_t day, size_t month, size_t year);

public:
    Date() = default;
    Date(const std::string& date);
    bool operator<=(const Date& other) const;
    bool operator<(const Date& other) const;
    bool operator==(const Date& other) const;
};
