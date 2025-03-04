#include "define_day.hpp"

int defineDay(int day, int month, int year) {
    const static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    year -= month < 3;
    return (year + year / 4 - year / 100 + year / 400 + t[month - 1] + day - 1) % 7;
}
