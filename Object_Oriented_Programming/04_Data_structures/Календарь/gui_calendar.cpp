#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>

#include "gui_calendar.hpp"
#include "number_of_days.hpp"
#include "define_day.hpp"

namespace {
    const std::string Days = "Mn Tu Wd Th Fr St Sn";

    std::string dayName(int n) {
        const std::string daysOfWeek[] = {"Mn", "Tu", "Wd", "Th", "Fr", "St", "Sn"};
        return daysOfWeek[n];
    }

    std::string getMonthName(int monthNumber) {
        const static std::string months[] = {"January", "February", "March",
                                             "April", "May", "June",
                                             "July", "August", "September",
                                             "October", "November", "December"};
        return months[monthNumber];
    }

    void filler(std::string& string, size_t n) {
        if (n == 0) return;
        string += std::string(n, ' ');
    }

    std::string inMiddle(const std::string& string, size_t width) {
        std::string empty;
        size_t mid = (width - string.length()) / 2;
        filler(empty, mid);
        empty += string;
        filler(empty, width - empty.length());
        return empty;
    }

    std::string strRepeater(const std::string& string, int n, const char* filler) {
        std::string repeated;
        for (int i = 0; i < n - 1; i++) repeated += string + filler;
        repeated += string;
        return repeated;
    }

    std::string templateOfNHorizMonth(int month, int year, bool everyYear, int n) {
        std::string out;
        std::string weeks = strRepeater(Days, n, "  ");
        if (!everyYear)
            out += inMiddle(std::to_string(year), 4 * Days.length()) + '\n';
        for (int i = 0; i < n; i++) {
            std::string monthWithYear = getMonthName(month + i);
            if (everyYear) {
                monthWithYear += " " + std::to_string(year);
            }
            out += inMiddle(monthWithYear, Days.length() + 2);
        }
        out += '\n' + weeks;
        return out;
    }

    int weeksAmount(int month, int year) {
        int count = 0;
        int current = defineDay(1, month + 1, year);
        int daysInMonth = numberOfDays(month, year);
        int currentPrevious = current - 1;
        for (int i = 1; i <= daysInMonth; i++)
            if (++currentPrevious > 6) {
                currentPrevious = 0;
                count++;
            }
        return count + 1;
    }

    std::string horizMonth(int month, int year) {
        std::ostringstream out;
        int current = defineDay(1, month + 1, year);
        int daysInMonth = numberOfDays(month, year);
        out << std::string(current * 3, ' ');
        for (int i = 1; i <= daysInMonth; i++) {
            out << std::right << std::setw(2) << i << " ";
            if (++current > 6) {
                current = 0;
                out << std::endl;
            }
        }
        std::string final = out.str();
        final.pop_back();
        int currentOfNext = defineDay(1, month + 2, year);
        int tabsAmount = 6 - currentOfNext;
        if (currentOfNext != 0)
            filler(final, (tabsAmount + 1) * 3);
        else if (month != 11)
            final.pop_back();
        return final;
    }

    std::string getNWeek(int month, int year, int n) {
        if (n > weeksAmount(month, year)) {
            return std::string(20, ' ');
        }
        std::string monthData = horizMonth(month, year);
        std::string weeks;
        int counter = 0;
        for (auto elem : monthData) {
            if (elem == '\n') {
                counter++;
                if (counter == n)
                    break;
                weeks.clear();
                continue;
            }
            weeks += elem;
        }
        if (n != weeksAmount(month, year)) weeks.pop_back();
        return weeks;
    }

    std::string printNHorizMonth(int month, int n, int year, bool everyYear) {
        std::string out;
        out += templateOfNHorizMonth(month, year, everyYear, n) + '\n';
        std::vector<int> weeksInfo;
        for (int i = 0; i < n; i++) weeksInfo.push_back(weeksAmount(month + i, year));
        long int lenWeekMax = weeksInfo[std::distance(weeksInfo.begin(), std::max_element(weeksInfo.begin(), weeksInfo.end()))];
        for (int j = 1; j <= lenWeekMax; j++) {
            for (int i = 0; i < n - 1; i++) {
                out += getNWeek(month + i, year, j) + "  ";
            }
            out += getNWeek(month + n - 1, year, j) + '\n';
        }
        out.pop_back();
        return out;
    }

    std::string vertMonth(int month, int year) {
        std::ostringstream out;
        int daysInMonth = numberOfDays(month, year);
        std::vector<std::vector<int>> daysOfWeek(7);
        for (int i = 1; i <= daysInMonth; i++) {
            int current = defineDay(i, month + 1, year);
            daysOfWeek[current].push_back(i);
        }
        bool wasFirstDay = false;
        for (auto& dayOfWeek : daysOfWeek) {
            if (!wasFirstDay) {
                bool firstDay = dayOfWeek[0] == 1;
                if (!firstDay)
                    out << "   ";
                if (firstDay)
                    wasFirstDay = true;
            }
            for (int day : dayOfWeek) {
                out << std::right << std::setw(2) << day << " ";
            }
            out << std::endl;
        }
        std::string final = out.str();
        final.pop_back();
        return final;
    }

    std::string getNDayOfWeek(int month, int year, int n) {
        std::string monthInfo = vertMonth(month, year);
        std::string temp;
        int counter = 0;
        for (auto elem : monthInfo) {
            if (elem == '\n') {
                counter++;
                if (counter == n)
                    break;
                temp.clear();
                continue;
            }
            temp += elem;
        }
        temp.pop_back();
        if (temp.length() >= 16)
            return temp;
        filler(temp, 17 - temp.length());
        return temp;
    }

    std::string templateOfNVertMonth(int month, int year, bool everyYear, int n) {
        std::string out;
        std::string months;
        for (int i = 0; i < n; i++) {
            std::string monthName = getMonthName(month + i);
            if (everyYear) {
                monthName += " " + std::to_string(year);
            }
            months += inMiddle(monthName, 19);
        }
        if (!everyYear) out += inMiddle(std::to_string(year), 75) + '\n';
        out += months;
        return out;
    }

    std::string printNVertMonth(int month, int n, int year, bool everyYear) {
        std::string out;
        out += templateOfNVertMonth(month, year, everyYear, n) + '\n';
        for (int j = 1; j <= 7; j++) {
            out += dayName(j - 1) + " ";
            for (int k = 0; k < n - 1; k++)
                out += getNDayOfWeek(month + k, year, j) + "  ";
            out += getNDayOfWeek(month + n - 1, year, j) + '\n';
        }
        out.pop_back();
        return out;
    }

    std::string horizontalPrint(int monthBeg, int yearBeg, int monthEnd, int yearEnd, bool yearEveryMonth, bool yearOnce) noexcept {
        std::ostringstream out;
        if (monthBeg == monthEnd && yearBeg == yearEnd) {
            out << printNHorizMonth(monthBeg - 1, 1, yearBeg, yearEveryMonth);
        } else if (monthBeg < monthEnd && yearBeg == yearEnd) {
            int monthAmount = monthEnd - monthBeg + 1;
            if (monthAmount < 4) {
                int year = yearBeg;
                out << printNHorizMonth(monthBeg - 1, monthAmount, year, yearEveryMonth) << std::endl;
            } else if ((monthAmount) % 4 == 0) {
                int year = yearBeg;
                int monthAmountPerLine = monthAmount / 4;
                for (int i = 0; i < monthAmountPerLine; i++) {
                    out << printNHorizMonth(monthBeg + 4 * i - 1, 4, year, yearEveryMonth) << std::endl;
                }
            } else if (monthAmount >= 5 && monthAmount <= 7) {
                int year = yearBeg;
                out << printNHorizMonth(monthBeg - 1, 4, year, yearEveryMonth) << std::endl;
                out << printNHorizMonth(monthBeg + 3, monthAmount - 4, year, yearEveryMonth);
            } else if (monthAmount >= 9 && monthAmount <= 11) {
                int year = yearBeg;
                out << printNHorizMonth(monthBeg - 1, 4, year, yearEveryMonth) << std::endl;
                out << printNHorizMonth(monthBeg + 3, 4, year, yearEveryMonth) << std::endl;
                out << printNHorizMonth(monthBeg + 3 + 4, monthAmount - 4 - 4, year, yearEveryMonth);
            }
        } else if (yearBeg < yearEnd) {
            int k = yearEnd - yearBeg - 1;
            std::string part1 = horizontalPrint(monthBeg, yearBeg, 12, yearBeg, yearEveryMonth, yearOnce);
            out << part1 << std::endl;
            if (k >= 1) {
                for (int i = yearBeg + 1; i < yearEnd; i++)
                    out << horizontalPrint(1, i, 12, i, yearEveryMonth, yearOnce) << std::endl;
            }
            std::string part2 = horizontalPrint(1, yearEnd, monthEnd, yearEnd, yearEveryMonth, yearOnce);
            out << part2 << std::endl;
        }
        return out.str();
    }

    std::string verticalPrint(int monthBeg, int yearBeg, int monthEnd, int yearEnd, bool yearEveryMonth, bool yearOnce) noexcept {
        std::ostringstream out;
        if (monthBeg == monthEnd && yearBeg == yearEnd) {
            out << printNVertMonth(monthBeg - 1, 1, yearBeg, yearOnce);
        } else if (monthBeg < monthEnd && yearBeg == yearEnd) {
            int monthAmount = monthEnd - monthBeg + 1;
            if (monthAmount < 4) {
                int year = yearBeg;
                out << printNVertMonth(monthBeg - 1, monthAmount, year, yearOnce) << std::endl;
            } else if ((monthAmount) % 4 == 0) {
                int year = yearBeg;
                int monthAmountPerLine = monthAmount / 4;
                for (int i = 0; i < monthAmountPerLine; i++) {
                    out << printNVertMonth(monthBeg + 4 * i - 1, 4, year, yearOnce) << std::endl;
                }
            } else if (monthAmount >= 5 && monthAmount <= 7) {
                int year = yearBeg;
                out << printNVertMonth(monthBeg - 1, 4, year, yearOnce) << std::endl;
                out << printNVertMonth(monthBeg + 3, monthAmount - 4, year, yearOnce);
            } else if (monthAmount >= 9 && monthAmount <= 11) {
                int year = yearBeg;
                out << printNVertMonth(monthBeg - 1, 4, year, yearOnce) << std::endl;
                out << printNVertMonth(monthBeg + 3, 4, year, yearOnce) << std::endl;
                out << printNVertMonth(monthBeg + 3 + 4, monthAmount - 4 - 4, year, yearOnce);
            }
        } else if (yearBeg < yearEnd) {
            int k = yearEnd - yearBeg - 1;
            std::string firstPart = verticalPrint(monthBeg, yearBeg, 12, yearBeg, yearEveryMonth, yearOnce);
            out << firstPart << std::endl;
            if (k >= 1) {
                for (int i = yearBeg + 1; i < yearEnd; i++)
                    out << verticalPrint(1, i, 12, i, yearEveryMonth, yearOnce) << std::endl;
            }
            std::string secondPart = verticalPrint(1, yearEnd, monthEnd, yearEnd, yearEveryMonth, yearOnce);
            out << secondPart << std::endl;
        }
        return out.str();
    }
}

std::string guiCalendar::Draw(bool yearEveryMonth, bool yearOnce, Date beginDate, Date endDate) {
    int monthBegin = beginDate.getMonth();
    int yearBegin = beginDate.getYear();
    int monthEnd = endDate.getMonth();
    int yearEnd = endDate.getYear();
    if (orient)
        return verticalPrint(monthBegin, yearBegin, monthEnd, yearEnd, yearEveryMonth, yearOnce);
    else
        return horizontalPrint(monthBegin, yearBegin, monthEnd, yearEnd, yearEveryMonth, yearOnce);
}
