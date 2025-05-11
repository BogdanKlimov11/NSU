#pragma once

#include "number_of_days.hpp"

int numberOfDays(int monthNumber, int year) {
    switch (monthNumber) {
        case 0:
        case 2:
        case 4:
        case 6:
        case 7:
        case 9:
        case 11:
            return 31;
        case 1:
            if (year % 400 == 0 || (year % 4 == 0 && year % 100 != 0))
                return 29;
            else
                return 28;
        case 3:
        case 5:
        case 8:
        case 10:
            return 30;
        default:
            return -1;
    }
}
