#include <gtest/gtest.h>

#include <fstream>

#include "parser.hpp"
#include "define_day.hpp"

using namespace std;

TEST(INPUT, RANGES) {
    ifstream input("testRanges.txt");
    vector<string> testInput = streamReader(input);
    ASSERT_TRUE("range 1 2021 12 2022 | horiz year_once" == testInput[0]);
    Range testRange = CreateRange(testInput);
    Range controlRange{};
    controlRange.dateBegin = Date(1, 1, 2021);
    controlRange.dateEnd = Date(31, 12, 2022);
    controlRange.type = RangeType::RANGE;
    controlRange.orient = false;
    controlRange.yearOnce = true;
    controlRange.yearEveryMonth = false;
    ASSERT_TRUE(testRange.type == controlRange.type);
    ASSERT_TRUE(testRange.orient == controlRange.orient);
    ASSERT_TRUE(testRange.dateEnd == controlRange.dateEnd);
    ASSERT_TRUE(testRange.dateBegin == controlRange.dateBegin);
    ASSERT_TRUE(testRange.yearOnce == controlRange.yearOnce);
    ASSERT_TRUE(testRange.yearEveryMonth == controlRange.yearEveryMonth);
    ASSERT_TRUE("rage 1 2021 12 2022 | horiz year_once" == testInput[1]);
}

TEST(DATE, TEST) {
    vector<string> DaysNames {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
    vector<string> MonthsNames {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
    Date testDate = Date(1, 1, 1);
    Date invalidDate1 = Date(-1, 1, 1);
    Date invalidDate2 = Date(1, -1, 1);
    Date invalidDate3 = Date(1, 1, -1);
    Date invalidDate4 = Date(-1, -1, -1);
    ASSERT_EQ(testDate.getDay(), 1);
    ASSERT_EQ(testDate.getYear(), 1);
    ASSERT_EQ(testDate.getMonth(), 1);
    ASSERT_TRUE(invalidDate1 == invalidDate2);
    ASSERT_TRUE(invalidDate2 == invalidDate3);
    ASSERT_TRUE(invalidDate3 == invalidDate4);
    ASSERT_TRUE(testDate.getMonthAsString() == MonthsNames[testDate.getMonth() - 1]);
    ASSERT_TRUE(testDate.getDayAsString() == DaysNames[defineDay(1, 1, 1)]);
    ASSERT_TRUE(invalidDate1.getMonthAsString() == "");
}
