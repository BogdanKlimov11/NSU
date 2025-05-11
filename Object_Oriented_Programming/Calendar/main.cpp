#include <iostream>
#include <fstream>

#include "gui_calendar.hpp"
#include "parser.hpp"
#include "days_count.hpp"

int run(const char FileName[]) {
    std::ifstream inp(FileName);
    std::istream& input = inp;
    Range testRange = CreateRange(streamReader(input));
    if (testRange.type == RangeType::UNDEFINED) {
        std::cout << "Wrong input!!!" << std::endl;
        return 0;
    }
    Date begin = Date(testRange.dateBegin.getDay(), testRange.dateBegin.getMonth(), testRange.dateBegin.getYear());
    Date end = Date(testRange.dateEnd.getDay(), testRange.dateEnd.getMonth(), testRange.dateEnd.getYear());
    guiCalendar testCal = guiCalendar(testRange.orient);
    std::cout << testCal.Draw(testRange.yearEveryMonth, testRange.yearOnce, testRange.dateBegin, testRange.dateEnd);
    std::cout << std::endl;
    auto testDays = daysCounter(begin, end);
    for (auto days : testDays) {
        std::cout << days << " ";
    }
    return 0;
}

int main(int argc, char* argv[]) {
    run(argv[1]);
}
