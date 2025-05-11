#include "days_count.hpp"
#include "define_day.hpp"

int rataDie(int d, int m, int y) {
    if (m < 3)
        y--, m += 12;
    return 365 * y + y / 4 - y / 100 + y / 400 + (153 * m - 457) / 5 + d - 306;
}

std::vector<int> daysCounter(Date beg, Date end) {
    std::vector<int> dayNamesAmount(7);
    int beginDay = defineDay(beg.getDay(), beg.getMonth(), beg.getYear());
    int endDay = defineDay(end.getDay(), end.getMonth(), end.getYear());
    int daysBetween = rataDie(end.getDay(), end.getMonth(), end.getYear()) -
                      rataDie(beg.getDay(), beg.getMonth(), beg.getYear());
    int v = daysBetween / 7;
    for (int i = 0; i < 7; i++) {
        dayNamesAmount[i] = v;
    }
    int delta = endDay - beginDay;
    if (delta != 0) {
        for (int i = 0; i <= delta; i++) {
            dayNamesAmount[beginDay + i] += 1;
        }
    }
    return dayNamesAmount;
}
