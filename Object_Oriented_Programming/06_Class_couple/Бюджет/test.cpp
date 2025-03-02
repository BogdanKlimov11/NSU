#include "gtest/gtest.h"

#include <tuple>

#include "budget.hpp"
#include "writer.hpp"

TEST(Parsers, ExpectedGetNext) {
    std::istringstream ss{
        "a:b 400\n"
        "a:d+h 1.0\n"
        "k:m   80\n"};
    ExpectedSpendingParser parser(ss);
    auto ab = parser.getNextItem();
    auto adh = parser.getNextItem();
    auto ak = parser.getNextItem();

    EXPECT_EQ(ab.second, true);
    EXPECT_EQ(adh.second, true);
    EXPECT_EQ(ak.second, true);

    EXPECT_EQ(ab.first.expectedValue, 400);
    EXPECT_EQ(adh.first.expectedValue, 1.0);
    EXPECT_EQ(ak.first.expectedValue, 80);

    EXPECT_EQ(ab.first.categoriesList.size(), 1);
    EXPECT_EQ(adh.first.categoriesList.size(), 2);
    EXPECT_EQ(ak.first.categoriesList.size(), 1);

    EXPECT_EQ(ab.first.categoriesList[0].category, "a");
    EXPECT_EQ(ab.first.categoriesList[0].subcategory, "b");
    EXPECT_EQ(adh.first.categoriesList[0].category, "a");
    EXPECT_EQ(adh.first.categoriesList[0].subcategory, "d");
    EXPECT_EQ(adh.first.categoriesList[1].category, "h");
    EXPECT_EQ(adh.first.categoriesList[1].subcategory, "");
    EXPECT_EQ(ak.first.categoriesList[0].category, "k");
    EXPECT_EQ(ak.first.categoriesList[0].subcategory, "m");
}

TEST(Parsers, ExpectedGetNextIvalid) {
    std::istringstream ss{
        "a:b \n"
        "aaaaaa a aaa aa\n"
        "a:b 400 2213\n"};
    ExpectedSpendingParser parser(ss);
    auto ab = parser.getNextItem();
    EXPECT_EQ(ab.second, true);
    EXPECT_EQ(ab.first.categoriesList.size(), 1);
    EXPECT_EQ(ab.first.categoriesList[0].category, "a");
    EXPECT_EQ(ab.first.categoriesList[0].subcategory, "b");
    EXPECT_EQ(ab.first.expectedValue, 400);
}

TEST(Parsers, ExpectedGetPath) {
    std::istringstream ss{"     "};
    ExpectedSpendingParser parser(ss);
    std::string s = "a:b+n:m";
    auto p = parser.getPath(s);
    EXPECT_EQ(p.size(), 2);
    EXPECT_EQ(p[0].category, "a");
    EXPECT_EQ(p[0].subcategory, "b");
    EXPECT_EQ(p[1].category, "n");
    EXPECT_EQ(p[1].subcategory, "m");
}

TEST(Parsers, RealGetNext) {
    std::istringstream ss{
        "10.2.2020 a:b 400\n"
        "11.4.2020 a:c 1.0\n"
        "13.7.2020 m:k   80\n"};
    RealSpendingParser parser(ss);
    auto ab = parser.getNextItem();
    auto ac = parser.getNextItem();
    auto mk = parser.getNextItem();
    EXPECT_EQ(ab.second, true);
    EXPECT_EQ(ac.second, true);
    EXPECT_EQ(mk.second, true);
    {
        Date d1("10.2.2020");
        Date d2("11.4.2020");
        Date d3("13.7.2020");
        EXPECT_TRUE(ab.first.getDate() <= d1);
        EXPECT_TRUE(ac.first.getDate() <= d2);
        EXPECT_TRUE(mk.first.getDate() <= d3);
        EXPECT_TRUE(ab.first.getDate() == d1);
        EXPECT_TRUE(ac.first.getDate() == d2);
        EXPECT_TRUE(mk.first.getDate() == d3);
    }
    {
        Date d1("11.2.2020");
        Date d2("11.10.2020");
        Date d3("13.9.2020");
        EXPECT_TRUE(ab.first.getDate() < d1);
        EXPECT_TRUE(ac.first.getDate() < d2);
        EXPECT_TRUE(mk.first.getDate() < d3);
    }
    EXPECT_EQ(ab.first.getValue(), 400.0);
    EXPECT_EQ(ac.first.getValue(), 1.0);
    EXPECT_EQ(mk.first.getValue(), 80.0);
    EXPECT_EQ(ab.first.getItem().category, "a");
    EXPECT_EQ(ab.first.getItem().subcategory, "b");
    EXPECT_EQ(ac.first.getItem().category, "a");
    EXPECT_EQ(ac.first.getItem().subcategory, "c");
    EXPECT_EQ(mk.first.getItem().category, "m");
    EXPECT_EQ(mk.first.getItem().subcategory, "k");
}

TEST(Parsers, RealGetNextIvalid) {
    std::istringstream ss{
        "11 a:b \n"
        "aaaaaa a aaa aa\n"
        "14.8.2021 a:b 400 2213\n"};
    RealSpendingParser parser(ss);
    auto ab = parser.getNextItem();
    Date d("14.8.2021");
    EXPECT_EQ(ab.second, true);
    EXPECT_TRUE(ab.first.getDate() == d);
    EXPECT_EQ(ab.first.getValue(), 400.0);
    EXPECT_EQ(ab.first.getItem().category, "a");
    EXPECT_EQ(ab.first.getItem().subcategory, "b");
}

TEST(Parsers, RealGetPath) {
    std::istringstream ss{"     "};
    RealSpendingParser parser(ss);
    std::string s = "a:b";
    auto p = parser.getPath(s);
    EXPECT_EQ(p.first, "a");
    EXPECT_EQ(p.second, "b");
}

TEST(Date, Methods) {
    {
        Date date1;
        Date date2;
        EXPECT_TRUE(Date("1.1.1") <= date1);
        EXPECT_TRUE(date1 == date2);
        EXPECT_TRUE(Date("20.12.2020") < date1);
    }
    {
        Date date("10.2.2010");
        EXPECT_TRUE(date <= Date("10.2.2020"));
        EXPECT_TRUE(date == Date("10.2.2010"));
        EXPECT_TRUE(date < Date("11.2.2010"));
    }
    {
        EXPECT_ANY_THROW(Date("10.14.2020"));
        EXPECT_ANY_THROW(Date("34.12.2020"));
        EXPECT_ANY_THROW(Date("29.2.1900"));
        EXPECT_ANY_THROW(Date("29.2.2007"));
        EXPECT_NO_THROW(Date("29.2.2004"));
    }
}

TEST(CategoryItem, Methods) {
    {
        CategoryItem item;
        CategoryItem item2{"aa", "bb"};
        CategoryItem item3{"", ""};
        EXPECT_TRUE(item < item2);
        EXPECT_TRUE(item == item3);
        EXPECT_TRUE(item != item2);
    }
    {
        CategoryItem item;
        CategoryItem item2{"aa", "bb"};
        CategoryItem item3{"aa", "nm"};
        CategoryItem item4{"al", "nm"};
        EXPECT_TRUE(item2 != item3);
        EXPECT_TRUE(item != item2);
        EXPECT_TRUE(item2 < item4);
        EXPECT_FALSE(item2 < item3);
        EXPECT_FALSE(item2 == item3);
        EXPECT_NO_THROW(item = item4);
        EXPECT_TRUE(item == item4);
    }
}

TEST(Categories, Methods) {
    CategoryItem item2{"aa", "bb"};
    CategoryItem item3{"aa", "nm"};
    CategoryItem item4{"ak", "nm"};
    {
        Categories cats1;
        Categories cats2;
        EXPECT_TRUE(cats1 == cats2);
        cats1.categoriesList.push_back(item2);
        cats2.categoriesList.push_back(item3);
        EXPECT_FALSE(cats1 < cats2);
        cats2.categoriesList.pop_back();
        cats2.categoriesList.push_back(item4);
        EXPECT_TRUE(cats1 < cats2);
    }
    {
        Categories cats1;
        Categories cats2;
        cats1.categoriesList.push_back(item2);
        cats1.categoriesList.push_back(item3);
        cats1.categoriesList.push_back(item4);
        EXPECT_TRUE(cats2 < cats1);
        EXPECT_NO_THROW(cats2 = cats1);
        EXPECT_TRUE(cats2 == cats1);
    }
    {
        Categories cats1;
        Categories cats2;
        cats2.expectedValue = 200;
        EXPECT_FALSE(cats2 == cats1);
        EXPECT_FALSE(cats1 < cats2);
        cats1.expectedValue = 200;
        EXPECT_TRUE(cats2 == cats1);
        EXPECT_FALSE(cats1 < cats2);
        cats1.categoriesList.push_back(item2);
        EXPECT_TRUE(cats2 < cats1);
        EXPECT_FALSE(cats2 == cats1);
    }
}

TEST(Spending, Methods) {
    CategoryItem item2{"aa", "bb"};
    Date date("10.2.2010");
    Date date2("14,2.2010");
    {
        Spending spend(date, "aa", "bb", 200);
        EXPECT_TRUE(spend.getDate() == date);
        EXPECT_TRUE(spend.getItem() == item2);
        EXPECT_TRUE(spend.getValue() == 200);
    }
    {
        Spending spend(date, "aa", "bb", 200);
        Spending spend1(date, "aa", "bb", 200);
        Spending spend2(date2, "aa", "bb", 200);
        Spending spend3(date, "kk", "", 200);
        EXPECT_TRUE(spend < spend2);
        EXPECT_TRUE(spend == spend1);
        EXPECT_TRUE(spend < spend3);
    }
    {
        Spending spend(date, "aa", "bb", 200);
        Spending spend1(date2, "kk", "", 7000);
        EXPECT_TRUE(spend < spend1);
        EXPECT_NO_THROW(spend = spend1);
        EXPECT_FALSE(spend < spend1);
        EXPECT_TRUE(spend == spend1);
    }
}

TEST(ItemInfo, Methods) {
    CategoryItem item2{"aa", "bb"};
    CategoryItem item3{"aa", "nm"};
    CategoryItem item4{"aK", "nm"};
    {
        itemInfo info1;
        itemInfo info2;
        EXPECT_TRUE(info1 == info2);
        EXPECT_FALSE(info1 < info2);
        info1.categoriesList.push_back(item2);
        info2.categoriesList.push_back(item3);
        info2.categoriesList.push_back(item3);
        EXPECT_TRUE(info1 < info2);
        EXPECT_FALSE(info1 == info2);
    }
    {
        itemInfo info1;
        itemInfo info2;
        EXPECT_TRUE(info1 == info2);
        EXPECT_FALSE(info1 < info2);
        info1.expected_price = 200;
        EXPECT_TRUE(info1 == info2);
        EXPECT_FALSE(info1 < info2);
        info1.percent = 8;
        EXPECT_TRUE(info1 == info2);
        EXPECT_FALSE(info1 < info2);
        info2.real_price = 100;
        EXPECT_TRUE(info1 == info2);
        EXPECT_FALSE(info1 < info2);
    }
}

TEST(Budget, Complex1) {
    std::istringstream expected{
        "a:b 400\n"
        "a:c+h:g 1000\n"
        "m   80\n"
        "other   100\n"};
    std::istringstream real{
        "10.2.2020 a:b 400\n"
        "11.4.2020 a:c 1.0\n"
        "12.4.2020 o 10\n"
        "13.7.2020 m:k   80\n"};
    RealSpendingParser parser_real(real);
    ExpectedSpendingParser parser_expected(expected);
    Date date1("10,2,2020");
    Date date2("14,4,2020");
    Budget b;
    std::vector<itemInfo> result;
    EXPECT_NO_THROW(b.expectedSpendingsFiller(parser_expected));
    EXPECT_NO_THROW(b.realSpendingsFiller(parser_real));
    EXPECT_NO_THROW(result = b.calculate(date1, date2));
    EXPECT_EQ(result.size(), 4);
    {
        CategoryItem item{"a", "b"};
        itemInfo info;
        info.categoriesList.push_back(item);
        info.expected_price = 400;
        info.real_price = 400;
        info.percent = 100;
        EXPECT_EQ(result[0], info);
    }
    {
        CategoryItem item1{"a", "c"};
        CategoryItem item2{"h", "g"};
        itemInfo info;
        info.categoriesList.push_back(item1);
        info.categoriesList.push_back(item2);
        info.expected_price = 1000;
        info.real_price = 1;
        info.percent = 0.1;
        EXPECT_EQ(result[1], info);
    }
    {
        CategoryItem item{"m", ""};
        itemInfo info;
        info.categoriesList.push_back(item);
        info.expected_price = 80;
        info.real_price = 0;
        info.percent = 0;
        EXPECT_EQ(result[2], info);
    }
    {
        EXPECT_EQ(result[3].categoriesList[0].category, "other");
        EXPECT_EQ(result[3].categoriesList[0].subcategory, "");
        EXPECT_EQ(result[3].expected_price, 100);
        EXPECT_EQ(result[3].real_price, 10);
        EXPECT_EQ(result[3].percent, 10);
    }
}

TEST(Budget, Complex2) {
    std::istringstream expected{
        "a:b 400\n"
        "d+h:g 1000\n"
        "m   80\n"};
    std::istringstream real{
        "10.2.2020 a:b 400\n"
        "11.4.2020 d 1.0\n"
        "12.4.2020 o 10\n"
        "13.7.2020 m:k   80\n"};
    RealSpendingParser parser_real(real);
    ExpectedSpendingParser parser_expected(expected);
    Date date1("10,2,2020");
    Date date2("14,4,2020");
    Budget b;
    std::vector<itemInfo> result;
    EXPECT_NO_THROW(b.expectedSpendingsFiller(parser_expected));
    EXPECT_NO_THROW(b.realSpendingsFiller(parser_real));
    EXPECT_NO_THROW(result = b.calculate(date1, date2));
    EXPECT_EQ(result.size(), 4);
    {
        CategoryItem item{"a", "b"};
        itemInfo info;
        info.categoriesList.push_back(item);
        info.expected_price = 400;
        info.real_price = 400;
        info.percent = 100;
        EXPECT_EQ(result[0], info);
    }
    {
        CategoryItem item1{"d", ""};
        CategoryItem item2{"h", "g"};
        itemInfo info;
        info.categoriesList.push_back(item1);
        info.categoriesList.push_back(item2);
        info.expected_price = 1000;
        info.real_price = 1;
        info.percent = 0.1;
        EXPECT_EQ(result[1], info);
    }
    {
        CategoryItem item{"m", ""};
        itemInfo info;
        info.categoriesList.push_back(item);
        info.expected_price = 80;
        info.real_price = 0;
        info.percent = 0;
        EXPECT_EQ(result[2], info);
    }
    {
        EXPECT_EQ(result[3].categoriesList[0].category, "other");
        EXPECT_EQ(result[3].categoriesList[0].subcategory, "");
        EXPECT_EQ(result[3].expected_price, 0);
        EXPECT_EQ(result[3].real_price, 10);
        EXPECT_EQ(result[3].percent, 0);
    }
}

TEST(Budget, BadExpectedStream) {
    std::istringstream expected{
        "a \n"
        ".....wer\n"
        "efsdgw\n"};
    std::istringstream real{
        "10.2.2020 a:b 400\n"
        "11.4.2020 a:c 1.0\n"
        "12.4.2020 o 10\n"
        "13.7.2020 m:k   80\n"};
    RealSpendingParser parser_real(real);
    ExpectedSpendingParser parser_expected(expected);
    Date date1("10,2,2020");
    Date date2("14,4,2020");
    Budget b;
    EXPECT_NO_THROW(b.expectedSpendingsFiller(parser_expected));
    EXPECT_NO_THROW(b.realSpendingsFiller(parser_real));
    EXPECT_ANY_THROW(b.calculate(date1, date2));
}

TEST(Budget, BadRealStream) {
    std::istringstream expected{
        "a:b 400\n"
        "a:c+h:g 1000\n"
        "m   80\n"};
    std::istringstream real{
        "10.2. 400\n"
        "a:c 1.0\n"
        "13 qweq\n"};
    RealSpendingParser parser_real(real);
    ExpectedSpendingParser parser_expected(expected);
    Date date1("10.2.2020");
    Date date2("14.4.2020");
    Budget b;
    EXPECT_NO_THROW(b.expectedSpendingsFiller(parser_expected));
    EXPECT_NO_THROW(b.realSpendingsFiller(parser_real));
    EXPECT_ANY_THROW(b.calculate(date1, date2));
}

TEST(Budget, BadDate) {
    std::istringstream expected{
        "a:b 400\n"
        "a:c+h:g 1000\n"
        "m   80\n"};
    std::istringstream real{
        "10.2. 400\n"
        "a:c 1.0\n"
        "13 qweq\n"};
    RealSpendingParser parser_real(real);
    ExpectedSpendingParser parser_expected(expected);
    Date date1;
    Date date2;
    Budget b;
    EXPECT_NO_THROW(b.expectedSpendingsFiller(parser_expected));
    EXPECT_NO_THROW(b.realSpendingsFiller(parser_real));
    EXPECT_ANY_THROW(b.calculate(date1, date2));
}

TEST(Writer, Right) {
    CategoryItem item1{"a", "c"};
    CategoryItem item2{"h", "g"};
    itemInfo info;
    info.categoriesList.push_back(item1);
    info.categoriesList.push_back(item2);
    info.expected_price = 1000;
    info.real_price = 1;
    info.percent = 0.1;
    std::vector<itemInfo> result{info};
    std::ostringstream oss;
    Writer writer(oss);
    writer.writeSpendings(result);
    EXPECT_EQ(oss.str(), "a:c+h:g    1000    1    0.1%\n");
}

TEST(Writer, Empty) {
    std::vector<itemInfo> result;
    std::ostringstream oss;
    Writer writer(oss);
    writer.writeSpendings(result);
    EXPECT_EQ(oss.str(), "");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
