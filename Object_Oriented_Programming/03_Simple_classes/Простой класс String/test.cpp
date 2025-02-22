#pragma once

#include "gtest.h"
#include "string.hpp"

TEST(ConstMethods, DataMet) {
    const char * str = "abcdefg";
    const char * stra = "aaa";
    const char * strcd = "cd";
    String str2("abc");
    String str3(str, 4);
    String str4(3, 'a');
    String str5(str2);
    String str6(str3, 2);
    
    for (size_t i = 0; i < str2.size(); i++) {
        EXPECT_EQ(*(str2.data() + i), *(str + i));
    }
    for (size_t i = 0; i < str3.size(); i++) {
        EXPECT_EQ(*(str3.data() + i), *(str + i));
    }
    for (size_t i = 0; i < str4.size(); i++) {
        EXPECT_EQ(*(str4.data() + i), *(stra + i));
    }
    for (size_t i = 0; i < str5.size(); i++) {
        EXPECT_EQ(*(str5.data() + i), *(str + i));
    }
    for (size_t i = 0; i < str6.size(); i++) {
        EXPECT_EQ(*(str6.data() + i), *(strcd + i));
    }
}

TEST(ConstMethods, CapacitySize) {
    String str2("abc");
    String str3("abcdefg", 4);
    String str4(3, 'a');
    String str5(str2);
    String str6(str3, 2);
    
    EXPECT_EQ(str2.capacity(), 4);
    EXPECT_EQ(str3.capacity(), 5);
    EXPECT_EQ(str4.capacity(), 4);
    EXPECT_EQ(str5.capacity(), 4);
    EXPECT_EQ(str6.capacity(), 3);
    
    EXPECT_EQ(str2.size(), 3);
    EXPECT_EQ(str3.size(), 4);
    EXPECT_EQ(str4.size(), 3);
    EXPECT_EQ(str5.size(), 3);
    EXPECT_EQ(str6.size(), 2);
}

TEST(ConstMethods, AtBackFrontOperator) {
    String str3("abcdefg", 4);
    
    EXPECT_EQ(str3.at(0), 'a');
    EXPECT_EQ(str3.at(3), 'd');
    EXPECT_ANY_THROW(str3.at(7));
    EXPECT_EQ(str3[0], 'a');
    EXPECT_EQ(str3[3], 'd');
    EXPECT_EQ(str3.front(), 'a');
    EXPECT_EQ(str3.back(), 'd');
    
    {
        const String str;
        ASSERT_EQ(str.at(0), '\0');
    }
    
    {
        const String str(nullptr);
        ASSERT_EQ(str.at(0), '\0');
    }
    
    {
        const String str(nullptr, String::npos);
        ASSERT_EQ(str.at(0), '\0');
    }
    
    {
        const String str("\0");
        ASSERT_EQ(str.at(0), '\0');
    }
}
TEST(NonConstMethods, AtBackFrontOperator) {
    String str3("abcdefg", 4);
    
    char &front = str3.front();
    char &back = str3.back();
    char &at = str3.at(2);
    front = 'k';
    back = 'l';
    at = 'o';
    str3[1] = 'j';
    EXPECT_EQ(str3.front(), 'k');
    EXPECT_EQ(str3.back(), 'l');
    EXPECT_EQ(str3.at(2), 'o');
    EXPECT_EQ(str3[1], 'j');
}

TEST(ConstMethods, Find) {
    String str2("abc");
    String str3("abcdefg", 4);
    
    EXPECT_EQ(str2.find('a'), 0);
    EXPECT_EQ(str3.find('a', 2), -1);
    EXPECT_EQ(str3.find('a', 0), 0);
    
    EXPECT_EQ(str3.find("bc"), 1);
    EXPECT_EQ(str3.find("bc", 1), 1);
    EXPECT_EQ(str3.find("bc", 3), -1);
    
    EXPECT_EQ(str3.find(str2), 0);
    EXPECT_EQ(str3.find(str2, 0), 0);
    EXPECT_EQ(str3.find(str2, 2), -1);
}

TEST(ConstMethods, Compare) {
    String str3("abcdefg", 4);
    String str2("abcd");
    String str1("cdab");
    String str4("abcde");
    
    EXPECT_EQ(str3.compare(str2), 0);
    EXPECT_EQ(str3.compare(str1), -1);
    EXPECT_EQ(str3.compare(str4), -1);
    EXPECT_EQ(str4.compare(str1), -1);
    EXPECT_EQ(str4.compare(str2), 1);
    EXPECT_EQ(str4.compare(""), 1);
    EXPECT_EQ(str4.compare("ad"), -1);
}

TEST(ConstMethods, EmptyClear) {
    String str2("abcd");
    EXPECT_FALSE(str2.empty());
    EXPECT_NO_THROW(str2.clear());
    EXPECT_TRUE(str2.empty());
}

TEST(ConstMethods, Substr) {
    String str2("abcd");
    std::stringstream tmp;
    tmp << str2.substr().data();
    EXPECT_EQ(tmp.str(), "abcd");
    tmp.str("");
    
    tmp << str2.substr(0).data();
    EXPECT_EQ(tmp.str(), "abcd");
    tmp.str("");
    
    tmp << str2.substr(0, 4).data();
    EXPECT_EQ(tmp.str(), "abcd");
    tmp.str("");
    
    tmp << str2.substr(2, 3).data();
    EXPECT_EQ(tmp.str(), "cd");
    tmp.str("");
    
    tmp << str2.substr(3).data();
    EXPECT_EQ(tmp.str(), "d");
    tmp.str("");
    
    tmp << str2.substr(3, 1).data();
    EXPECT_EQ(tmp.str(), "d");
    tmp.str("");
    
    tmp << str2.substr(3, 0).data();
    EXPECT_EQ(tmp.str(), "");
    tmp.str("");
    EXPECT_ANY_THROW(str2.substr(7, 8).data());
}


TEST(NonConstMethods, Reserve) {
    String str2("abcd");
    EXPECT_NO_THROW(str2.reserve(7));
    EXPECT_EQ(str2.capacity(), 7);
    EXPECT_NO_THROW(str2.reserve(5));
    EXPECT_EQ(str2.capacity(), 5);
    EXPECT_NO_THROW(str2.reserve(3));
    EXPECT_EQ(str2.capacity(), 5);
    EXPECT_NO_THROW(str2.reserve(0));
    EXPECT_EQ(str2.capacity(), 5);
}

TEST(NonConstMethods, Swap) {
    String str2("abcd");
    String tmp2("abcd");
    String str("string");
    String tmp("string");
    String str1("");
    String tmp1("");
    EXPECT_NO_THROW(str2.swap(str));
    EXPECT_EQ(str2.compare("string"), 0);
    EXPECT_EQ(str.compare("abcd"), 0);
    EXPECT_NO_THROW(str.swap(str1));
    EXPECT_EQ(str1.compare("abcd"), 0);
    EXPECT_EQ(str.compare(""), 0);
}

TEST(NonConstMethods, Erase) {
    String str("abcd");
    EXPECT_NO_THROW(str.erase(0, 2));
    EXPECT_EQ(str.compare("cd"), 0);
    EXPECT_ANY_THROW(str.erase(7, 2));
    EXPECT_EQ(str.compare("cd"), 0);
    EXPECT_NO_THROW(str.erase());
    EXPECT_EQ(str.compare(""), 0);
}

TEST(NonConstMethods, Replace) {
    String str("abcd");
    String str1("cd");
    String str2("worddd");
    
    //String& replace(size_t pos, size_t len, const String& str);
    EXPECT_NO_THROW(str.replace(0, 2, str1));
    EXPECT_EQ(str.compare("cdcd"), 0);
    EXPECT_ANY_THROW(str.replace(11, 2, str1));
    EXPECT_EQ(str.compare("cdcd"), 0);
    EXPECT_NO_THROW(str.replace(0, str.npos, str2));
    EXPECT_EQ(str.compare("worddd"), 0);
    
    //String& replace(size_t pos, size_t len, const char* str);
    EXPECT_NO_THROW(str1.replace(0, 2, "tt"));
    EXPECT_EQ(str1.compare("tt"), 0);
    EXPECT_NO_THROW(str1.replace(0, 4, "aarr"));
    EXPECT_EQ(str1.compare("aarr"), 0);
    EXPECT_ANY_THROW(str1.replace(11, 4, "ooooo"));
    EXPECT_EQ(str1.compare("aarr"), 0);
    
    //String& replace(size_t pos, size_t len, size_t n, char c);
    EXPECT_EQ(str2.compare("worddd"), 0);
    EXPECT_NO_THROW(str2.replace(0, 2, 4,'p'));
    EXPECT_EQ(str2.compare("pppprddd"), 0);
    EXPECT_NO_THROW(str2.replace(0, str1.npos, 7, 'u'));
    EXPECT_EQ(str2.compare("uuuuuuu"), 0);
}

TEST(NonConstMethods, Insert) {
    String str("abcd");
    String str1("efg");
    EXPECT_NO_THROW(str.insert(4, str1));
    EXPECT_EQ(str.compare("abcdefg"), 0);
    EXPECT_NO_THROW(str.insert(0, str1));
    EXPECT_EQ(str.compare("efgabcdefg"), 0);
    EXPECT_ANY_THROW(str.insert(20, str1));
    
    EXPECT_NO_THROW(str1.insert(0, "ab"));
    EXPECT_EQ(str1.compare("abefg"), 0);
    EXPECT_ANY_THROW(str1.insert(10, "oop"));
    
    EXPECT_NO_THROW(str1.insert(0, 1,'a'));
    EXPECT_EQ(str1.compare("aabefg"), 0);
    EXPECT_NO_THROW(str1.insert(3, 3, 'a'));
    EXPECT_EQ(str1.compare("aabaaaefg"), 0);
    EXPECT_ANY_THROW(str1.insert(20, 3, 'm'));
}

TEST(NonConstMethods, Operators) {
    String str("ab");
    String str1("ef");
    EXPECT_NO_THROW(str = str1);
    EXPECT_EQ(str.compare("ef"), 0);
    EXPECT_NO_THROW(str = "str");
    EXPECT_EQ(str.compare("str"), 0);
    
    EXPECT_NO_THROW(str += str1);
    EXPECT_EQ(str.compare("stref"), 0);
    EXPECT_NO_THROW(str += "cd");
    EXPECT_EQ(str.compare("strefcd"), 0);
    EXPECT_NO_THROW(str += 'e');
    EXPECT_EQ(str.compare("strefcde"), 0);
}

TEST(CopyOnWrite, CountRef) {
    String str("ab");
    EXPECT_EQ(str.countRef(), 1);
    String str1(str);
    
    EXPECT_EQ(str.countRef(), 2);
    String s("");
    s = str;
    EXPECT_EQ(str.countRef(), 3);
    EXPECT_NO_THROW(str1.clear());
    EXPECT_EQ(str.countRef(), 2);
    EXPECT_NO_THROW(s.clear());
    EXPECT_EQ(str.countRef(), 1);
}

const char TEST_STRING[] = "Test string";
const size_t TEST_SIZE = sizeof(TEST_STRING) / sizeof(TEST_STRING[0]) - 1;

TEST(NonConstAt1, ExceptionEmptyString) {
    {
        String str;
        ASSERT_THROW(str.at(0), std::out_of_range);
    }
    
    {
        String str(nullptr);
        ASSERT_THROW(str.at(0), std::out_of_range);
    }
    
    {
        String str(nullptr, String::npos);
        ASSERT_THROW(str.at(0), std::out_of_range);
    }
    
    {
        String str("\0");
        ASSERT_THROW(str.at(0), std::out_of_range);
    }
    
    {
        String str(TEST_STRING, 0);
        ASSERT_THROW(str.at(0), std::out_of_range);
    }
    
    {
        String str(TEST_STRING);
        String copy(str, TEST_SIZE);
        ASSERT_THROW(copy.at(0), std::out_of_range);
    }
    
    {
        String s;
        EXPECT_THROW(s.at(s.size()), std::out_of_range);
    }
}

TEST(NonConstAt1, ExceptionString) {
    String str(TEST_STRING);
    ASSERT_THROW(str.at(TEST_SIZE), std::out_of_range);
    ASSERT_THROW(str.at(String::npos), std::out_of_range);
}

TEST(OperatorPlusAssignment1, PlusEmptyStings) {
    String str;
    str += nullptr;
    ASSERT_TRUE(str.empty());
    str += "";
    ASSERT_TRUE(str.empty());
    str += "\0";
    ASSERT_TRUE(str.empty());
}

TEST(Find1, EmptyString) {
    {
        String str;
        ASSERT_TRUE(String::npos == str.find('\0'));
    }
    
    {
        String str("");
        ASSERT_TRUE(String::npos == str.find('\0'));
    }
    
    {
        String str("\0");
        ASSERT_TRUE(String::npos == str.find('\0'));
    }
    
    {
        String str = nullptr;
        ASSERT_TRUE(String::npos == str.find('\0'));
    }
}

TEST(Insert1, EmptyStrings) {
    String emptyString;
    String str(TEST_STRING);
    EXPECT_NO_THROW(str.insert(0, nullptr));
    ASSERT_TRUE(str.size() == TEST_SIZE);
    EXPECT_NO_THROW(str.insert(0, ""));
    ASSERT_TRUE(str.size() == TEST_SIZE);
    EXPECT_NO_THROW(str.insert(0, "\0"));
    ASSERT_TRUE(str.size() == TEST_SIZE);
    EXPECT_NO_THROW(str.insert(0, 0, '\0'));
    ASSERT_TRUE(str.size() == TEST_SIZE);
    EXPECT_NO_THROW(str.insert(0, emptyString));
    ASSERT_TRUE(str.size() == TEST_SIZE);
}

TEST(OperatorPlusAssignment1, PlusEmptyStringForCopy) {
    String str(TEST_STRING);
    const String copy(str);
    str += nullptr;
    ASSERT_TRUE(copy.countRef() == 2);
    str += "";
    ASSERT_TRUE(copy.countRef() == 2);
    str += "\0";
    ASSERT_TRUE(copy.countRef() == 2);
}

TEST(Insert1, EmptyStringsForCopy) {
    String emptyString;
    String str(TEST_STRING);
    const String copy(str);
    str.insert(0, nullptr);
    ASSERT_TRUE(copy.countRef() == 2);
    str.insert(0, "");
    ASSERT_TRUE(copy.countRef() == 2);
    str.insert(0, "\0");
    ASSERT_TRUE(copy.countRef() == 2);
    str.insert(0, 0, '\0');
    ASSERT_TRUE(copy.countRef() == 2);
    str.insert(0, emptyString);
    ASSERT_TRUE(copy.countRef() == 2);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
