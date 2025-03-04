#include <gtest/gtest.h>

#include "binary_tree.hpp"

class Custom {
    double x_{};
    std::string str_{};

public:
    Custom() = default;
    explicit Custom(double&& x, std::string&& str) : x_(x), str_(str) {}
    bool operator==(const Custom& another) const { return x_ == another.x_ && str_ == another.str_; }
    bool operator!=(const Custom& another) const { return !(*this == another); }
    bool operator<(const Custom& another) const { return x_ < another.x_; }
};

TEST(Constructor, Empty) {
    btree<int, std::string> t;
    ASSERT_TRUE(t.empty());
    ASSERT_EQ(t.size(), 0);
    ASSERT_TRUE(t.begin() == t.end());
}

TEST(Constructor, CustomComp) {
    btree<int, std::string> t1;
    t1[3] = "three";
    t1[1] = "one";
    t1[5] = "five";
    std::string buf;
    for (auto i = t1.begin(); i != t1.end(); ++i) {
        buf += i->second;
    }
    ASSERT_EQ(buf, "onethreefive");

    btree<int, std::string, std::greater<>> t2;
    t2[3] = "three";
    t2[1] = "one";
    t2[5] = "five";
    buf.clear();
    for (auto i = t2.begin(); i != t2.end(); ++i) {
        buf += i->second;
    }
    ASSERT_EQ(buf, "fivethreeone");
}

TEST(Constructor, Copy) {
    btree<int, Custom> t1;
    t1[1] = Custom(0.5, "one");
    t1[2] = Custom(0.1, "two");
    t1[3] = Custom(0.4, "three");
    t1[4] = Custom(0.11, "five");
    auto t2 = btree<int, Custom>(t1);
    ASSERT_TRUE(!t2.empty());
    ASSERT_EQ(t2.size(), 4);
    for (auto i = t1.begin(); i != t1.end(); ++i) {
        ASSERT_EQ(t1.at(i->first), t2.at(i->first));
    }
}

TEST(Constructor, Move) {
    btree<Custom, int> t1;
    const auto key_0 = Custom(0, "zero");
    const auto key_1 = Custom(0.1, "one");
    const auto key_2 = Custom(0.2, "two");
    t1[key_2] = 2;
    t1[key_1] = 1;
    t1[key_0] = 0;
    auto t2 = btree<Custom, int>(std::move(t1));
    ASSERT_TRUE(t1.empty());
    ASSERT_EQ(t2[key_2], 2);
    ASSERT_EQ(t2[key_1], 1);
    ASSERT_EQ(t2[key_0], 0);
}

TEST(Operator, CopyAssigment) {
    btree<std::string, std::string> t1;
    t1["One"] = "_one";
    t1["Two"] = "_two";
    auto& t2 = t1;
    ASSERT_TRUE(!t2.empty());
    ASSERT_EQ(t2.size(), 2);
    ASSERT_EQ((t2.find("One"))->second, "_one");
    for (auto i = t1.begin(); i != t1.end(); ++i) {
        ASSERT_EQ(t1.at(i->first), t2.at(i->first));
    }
}

TEST(Operator, MoveAssigment) {
    btree<Custom, std::string> t1;
    const auto key_0 = Custom(0, "zero");
    const auto key_1 = Custom(0.1, "one");
    const auto key_2 = Custom(0.2, "two");
    t1[key_2] = "str2";
    t1[key_1] = "str1";
    t1[key_0] = "str0";
    auto t2 = std::move(t1);
    ASSERT_TRUE(t1.empty());
    ASSERT_EQ(t2[key_2], "str2");
    ASSERT_EQ(t2[key_1], "str1");
    ASSERT_EQ(t2[key_0], "str0");
    t1 = std::move(t2);
    ASSERT_TRUE(t2.empty());
    ASSERT_EQ(t1[key_2], "str2");
    ASSERT_EQ(t1[key_1], "str1");
    ASSERT_EQ(t1[key_0], "str0");
}

TEST(Operator, Brackets) {
    const auto key_1 = Custom(0.2, "one");
    const auto key_2 = Custom(0.1, "zero");
    btree<Custom, std::string> t;
    t[key_1] = "_one";
    t[key_2] = "_two";
    t[key_1] = "_three";
    ASSERT_TRUE(!t.empty());
    ASSERT_EQ(t.size(), 2);
    ASSERT_EQ(t[key_1], "_three");
    ASSERT_EQ(t[key_2], "_two");
}

TEST(Operator, Equality) {
    {
        btree<std::string, std::string> t1;
        t1["Two"] = "_two";
        t1["One"] = "_one";
        t1["Three"] = "_three";
        const auto& t2 = t1;
        ASSERT_TRUE(t1 == t2);
    }
    {
        btree<std::string, std::string> t1;
        btree<std::string, std::string> t2;
        t1["One"] = "_one";
        t2["Two"] = "_two";
        ASSERT_TRUE(t1 != t2);
    }
}

TEST(Iterator, Begin) {
    btree<std::string, std::string> t;
    t["One"] = "_one";
    ASSERT_EQ(t.begin()->second, "_one");
    t["Two"] = "_two";
    ASSERT_EQ(t.begin()->second, "_one");
}

TEST(Iterator, ConstBegin) {
    btree<std::string, std::string> t;
    t["One"] = "_one";
    const auto& t1 = t;
    ASSERT_EQ(t1.begin()->second, "_one");
    t["Two"] = "_two";
    const auto& t2 = t;
    ASSERT_EQ(t2.begin()->second, "_one");
}

TEST(Iterator, End) {
    btree<std::string, std::string> t;
    t["One"] = "_one";
    t["Two"] = "_two";
    t["Three"] = "_three";
    std::string buf;
    for (auto i = t.begin(); i != t.end(); ++i) {
        buf += i->second;
    }
    ASSERT_EQ(buf, "_one_three_two");
}

TEST(Iterator, ConstEnd) {
    btree<std::string, std::string> t1;
    t1["One"] = "_one";
    t1["Two"] = "_two";
    t1["Three"] = "_three";
    std::string buf;
    const auto& t2 = t1;
    for (auto i = t2.begin(); i != t2.end(); ++i) {
        buf += i->second;
    }
    ASSERT_EQ(buf, "_one_three_two");
}

TEST(Method, At) {
    btree<std::string, int> t;
    ASSERT_TRUE(t.empty());
    ASSERT_EQ(t.size(), 0);
    EXPECT_THROW(t.at("one"), std::out_of_range);
    ASSERT_TRUE(t.empty());
    ASSERT_EQ(t.size(), 0);
    t["one"] = 1;
    ASSERT_TRUE(!t.empty());
    ASSERT_EQ(t.size(), 1);
    ASSERT_EQ(t.at("one"), 1);
}

TEST(Method, InsertEmpty) {
    btree<std::string, int> t;
    ASSERT_TRUE(t.empty());
    ASSERT_EQ(t.size(), 0);
    ASSERT_TRUE(t.begin() == t.end());
    auto it = t.insert(std::make_pair("two", 2));
    ASSERT_EQ(it.first, t.begin());
    ASSERT_EQ(it.second, true);
    ASSERT_TRUE(!t.empty());
    ASSERT_EQ(t.size(), 1);
    ASSERT_EQ(t.begin()->second, 2);
}

TEST(Method, InsertExsistingKey) {
    btree<std::string, int> t;
    t["two"] = 2;
    auto it = t.insert(std::make_pair("two", 1));
    ASSERT_EQ(it.first, t.begin());
    ASSERT_EQ(it.second, false);
    ASSERT_EQ(t.size(), 1);
    ASSERT_EQ(t.begin()->second, 2);
    it = t.insert(std::make_pair("two", 33));
    ASSERT_EQ(it.first, t.begin());
    ASSERT_EQ(it.second, false);
    ASSERT_EQ(t.size(), 1);
    ASSERT_EQ(t.begin()->second, 2);
}

TEST(Method, InsertNonExsistingKey) {
    btree<std::string, int> t;
    t["two"] = 2;
    auto it = t.insert(std::make_pair("one", 1));
    ASSERT_EQ(it.first, t.begin());
    ASSERT_EQ(it.second, true);
    ASSERT_EQ(t.size(), 2);
    ASSERT_EQ(t.begin()->second, 1);
    it = t.insert(std::make_pair("a", 33));
    ASSERT_EQ(it.first, t.begin());
    ASSERT_EQ(it.second, true);
    ASSERT_EQ(t.size(), 3);
    ASSERT_EQ(t.begin()->second, 33);
}

TEST(Method, KeyEraseEmpty) {
    btree<int, int> t;
    t.erase(6);
    ASSERT_TRUE(t.empty());
}

TEST(Method, KeyEraseExsistingItem) {
    btree<int, int> t;
    t[5] = 6;
    t.erase(5);
    ASSERT_TRUE(t.empty());
    t.insert(std::make_pair(1, 4));
    t.insert(std::make_pair(1, 5));
    t.insert(std::make_pair(1, 4));
    t.erase(1);
    ASSERT_TRUE(t.empty());
}

TEST(Method, KeyEraseNonExsistingItem) {
    btree<int, int> t;
    t[5] = 6;
    t.erase(6);
    ASSERT_EQ(t.size(), 1);
}

TEST(Method, PosEraseEmpty) {
    btree<int, int> t;
    ASSERT_EQ(t.begin(), t.end());
    t.erase(t.begin());
    ASSERT_EQ(t.begin(), t.end());
}

TEST(Method, PosEraseNonEmpty) {
    btree<int, int> t;
    t[2] = 3;
    t[0] = 1;
    t[1] = 2;
    t.insert(std::make_pair(2, 4));
    t[3] = 5;
    t.erase(t.find(2));
    ASSERT_EQ(t.size(), 3);
    ASSERT_EQ(t.find(2), t.end());
}

TEST(Method, IterEraseEmpty) {
    btree<int, int> t;
    t.erase(t.begin(), t.end());
    ASSERT_TRUE(t.empty());
    ASSERT_EQ(t.size(), 0);
}

TEST(Method, IterEraseNonEmpty) {
    btree<int, int> t;
    t[1] = 2;
    t[0] = 1;
    t[2] = 3;
    t.insert(std::make_pair(2, 4));
    t[3] = 5;
    t.erase(t.find(2), t.find(3));
    ASSERT_EQ(t.size(), 3);
    ASSERT_EQ(t.begin()->second, 1);
    ASSERT_EQ(++(t.begin())->second, 2);
}

TEST(Method, FindNonConst) {
    btree<int, int> t;
    t[0] = 1;
    t[1] = 2;
    t[2] = 33;
    t.insert(std::make_pair(2, 4));
    t[3] = 5;
    auto it = t.find(2);
    ASSERT_EQ(it->first, 2);
    ASSERT_EQ(it->second, 33);
    it->second = 3;
    it = t.find(2);
    ASSERT_EQ(it->first, 2);
    ASSERT_EQ(it->second, 3);
}

TEST(Method, FindConst) {
    btree<int, int> t;
    t[0] = 1;
    t[1] = 2;
    t[2] = 3;
    t.insert(std::make_pair(2, 4));
    t[3] = 5;
    const auto t_copy = btree<int, int>(t);
    auto it = t_copy.find(3);
    ASSERT_EQ(it->first, 3);
    ASSERT_EQ(it->second, 5);
}

TEST(Method, ClearEmpty) {
    btree<std::string, int> t;
    ASSERT_TRUE(t.empty());
    ASSERT_EQ(t.size(), 0);
    t.clear();
    ASSERT_TRUE(t.empty());
    ASSERT_EQ(t.size(), 0);
}

TEST(Method, ClearNonEmpty) {
    btree<int, int> t;
    t[0] = 1;
    t[1] = 2;
    t[2] = 3;
    t.insert(std::make_pair(2, 4));
    t[3] = 5;
    ASSERT_TRUE(!t.empty());
    ASSERT_EQ(t.size(), 4);
    t.clear();
    ASSERT_TRUE(t.empty());
    ASSERT_EQ(t.size(), 0);
    t[-5] = 1;
    t[-10] = 0;
    t[1] = 2;
    t[-1] = 5;
    ASSERT_EQ(t.begin()->second, 0);
    ASSERT_EQ(t.begin()->first, -10);
    ASSERT_TRUE(!t.empty());
    ASSERT_EQ(t.size(), 4);
}

TEST(Method, SwapEmpty) {
    btree<int, int> t1, t2;
    t2[1] = 3;
    t1.swap(t2);
    ASSERT_TRUE(t2.empty());
    ASSERT_EQ(t1.begin()->second, 3);
}

TEST(Method, SwapNonEmpty) {
    btree<std::string, int> t1, t2;
    t1["one"] = 1;
    t2["two"] = 2;
    t1.swap(t2);
    ASSERT_EQ(t1.begin()->second, 2);
    ASSERT_EQ(t2.begin()->second, 1);
    ASSERT_EQ(t1.begin()->first, "two");
    ASSERT_EQ(t2.begin()->first, "one");
}
