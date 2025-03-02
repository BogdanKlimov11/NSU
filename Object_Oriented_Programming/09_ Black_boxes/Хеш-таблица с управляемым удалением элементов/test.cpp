#include "gtest/gtest.h"

#include <thread>
#include <chrono>

#include "hash_table.hpp"
#include "time_strategy.hpp"
#include "count_strategy.hpp"

TEST(CountStrategyTest, Pair) {
    CountStrategy<int> csf(2);
    HashTable<int, int> htf(csf);
    htf.insert(1, 7);
    EXPECT_EQ(htf[1], 7);
    EXPECT_EQ(htf.at(1), 7);
    EXPECT_ANY_THROW(htf.at(1));
    htf.insert(1, 4);
    EXPECT_EQ(htf.at(1), 4);
    EXPECT_EQ(htf.erase(1), 1);
    EXPECT_ANY_THROW(htf.at(4));
}

TEST(CountStrategyTest, FewPairs) {
    CountStrategy<int> csf(2);
    HashTable<int, int> htf(csf);
    htf.insert(1, 7);
    htf.insert(3, 9);
    htf.insert(4, 11);
    EXPECT_EQ(htf[1], 7);
    EXPECT_EQ(htf[1], 7);
    EXPECT_ANY_THROW(htf.at(1));
    EXPECT_EQ(htf.at(3), 9);
    EXPECT_EQ(htf.at(3), 9);
    EXPECT_ANY_THROW(htf.at(3));
    EXPECT_EQ(htf.at(4), 11);
    EXPECT_EQ(htf.erase(4), 1);
    EXPECT_ANY_THROW(htf.at(4));
}

TEST(TimeStrategyTest, Pair) {
    TimeStrategy<int> tsf(1);
    HashTable<int, int> httf(tsf);
    httf.insert(7, 9);
    EXPECT_EQ(httf.at(7), 9);
    EXPECT_EQ(httf[7], 9);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    EXPECT_ANY_THROW(httf.at(7));
    httf.insert(7, 10);
    EXPECT_EQ(httf.at(7), 10);
    EXPECT_EQ(httf.erase(7), 1);
    EXPECT_ANY_THROW(httf.at(4));
}

TEST(TimeStrategyTest, FewPairs) {
    TimeStrategy<int> tsf(3);
    HashTable<int, int> httf(tsf);
    httf.insert(7, 9);
    httf.insert(1, 2);
    httf.insert(3, 4);
    EXPECT_EQ(httf.at(7), 9);
    EXPECT_EQ(httf.at(1), 2);
    EXPECT_EQ(httf.at(3), 4);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    EXPECT_EQ(httf[7], 9);
    EXPECT_EQ(httf[1], 2);
    EXPECT_EQ(httf.erase(3), 1);
    EXPECT_ANY_THROW(httf.at(3));
    std::this_thread::sleep_for(std::chrono::seconds(3));
    EXPECT_ANY_THROW(httf.at(7));
    EXPECT_ANY_THROW(httf.at(1));
}

TEST(CountStrategyTest, Iterators) {
    CountStrategy<int> cs(2);
    HashTable<int, int> ht(cs);
    ht.insert(8, 7);
    auto it = ht.begin();
    int i = 0;
    for (it; it != ht.end(); ++it) {
        i++;
    }
    auto iter = ht.begin();
    EXPECT_EQ(i, 1);
    EXPECT_NO_THROW(*iter);
    EXPECT_NO_THROW(*iter);
    EXPECT_ANY_THROW(*iter);
}

TEST(CountStrategyTest, FewPairsIterators) {
    CountStrategy<int> cs(1);
    HashTable<int, int> ht(cs);
    for (int i = 0; i < 4; i++) {
        ht.insert(i, i + 10);
    }
    auto it = ht.begin();
    int i = 0;
    for (it; it != ht.end(); ++it) {
        i++;
    }
    EXPECT_EQ(i, 4);
    auto iter = ht.begin();
    for (iter; iter != ht.end(); ++iter) {
        EXPECT_NO_THROW(*iter);
    }
    EXPECT_ANY_THROW(ht.at(0));
    EXPECT_ANY_THROW(ht.at(1));
    EXPECT_ANY_THROW(ht.at(2));
    EXPECT_ANY_THROW(ht.at(3));
}

TEST(TimeStrategyTest, Iterators) {
    TimeStrategy<int> cs(1);
    HashTable<int, int> ht(cs);
    ht.insert(8, 7);
    auto it = ht.begin();
    int i = 0;
    for (it; it != ht.end(); ++it) {
        i++;
    }
    auto iter = ht.begin();
    EXPECT_EQ(i, 1);
    EXPECT_NO_THROW(*iter);
    EXPECT_NO_THROW(*iter);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    EXPECT_ANY_THROW(*iter);
}

TEST(TimeStrategyTest, FewPairsIterators) {
    TimeStrategy<int> tsf(3);
    HashTable<int, int> ht(tsf);
    for (int i = 0; i < 4; i++) {
        ht.insert(i, i + 10);
    }
    auto it = ht.begin();
    int i = 0;
    for (it; it != ht.end(); ++it) {
        EXPECT_NO_THROW(*it);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto iter = ht.begin();
    for (iter; iter != ht.end(); ++iter) {
        EXPECT_NO_THROW(*iter);
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    auto iter4 = ht.begin();
    EXPECT_ANY_THROW(*iter4);
}
