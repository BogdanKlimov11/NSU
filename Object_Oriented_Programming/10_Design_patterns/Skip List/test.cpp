#include "gtest/gtest.h"

#include "skip_list.hpp"

const int const_size = 100;

template<typename Key, typename Value>
std::ostream& operator<<(std::ostream& o, const SkipList<Key, Value>& list) {
    o << "{ ";
    for (auto it = list.begin(); it != list.end(); ++it) {
        o << "{" << (*it).first << "," << (*it).second << "} ";
    }
    o << " }\n";
    return o;
}

TEST(Constructors, Empty) {
    SkipList<int, int> l1;
    EXPECT_TRUE(l1.empty());
    EXPECT_EQ(l1.size(), 0);
}

TEST(Insert, One) {
    SkipList<const char, int> l1;
    l1.insert({ 'A', 100 });
    EXPECT_EQ(l1.at('A'), 100);
    EXPECT_EQ(l1.size(), 1);
    auto begin2 = l1.begin();
    EXPECT_EQ((*begin2).second, 100);
}

TEST(Insert, FewElems) {
    SkipList<const int, int> list;
    list.insert({ 1, 10 });
    EXPECT_EQ(list.at(1), 10);
    EXPECT_EQ(list.size(), 1);
    list.insert({ 2, 20 });
    EXPECT_EQ(list.size(), 2);
    list.insert({ 3, 30 });
    EXPECT_EQ(list.size(), 3);
}

TEST(At, FewElements) {
    SkipList<int, int> l1;
    l1.insert({ 1, 10 });
    l1.insert({ 2, 20 });
    l1.insert({ 3, 30 });
    EXPECT_EQ(l1.at(1), 10);
    EXPECT_EQ(l1.at(2), 20);
    EXPECT_EQ(l1.at(3), 30);
    EXPECT_ANY_THROW(l1.at(267));
}

TEST(Skiplist, CopyConstructor) {
    SkipList<const int, int> list;
    list.insert({ 1, 10 });
    list.insert({ 2, 20 });
    list.insert({ 3, 30 });
    EXPECT_EQ(list.size(), 3);
    SkipList<const int, int> copy(list);
    EXPECT_EQ(copy.size(), 3);
}

TEST(Skiplist, Swap) {
    SkipList<const int, int> list;
    SkipList<const int, int> list1;
    list.insert({ 1, 10 });
    list.insert({ 2, 20 });
    list.insert({ 3, 30 });
    list1.insert({ 4, 40 });
    list1.insert({ 5, 50 });
    EXPECT_EQ(list.size(), 3);
    EXPECT_EQ(list1.size(), 2);
    EXPECT_NO_THROW(list.swap(list1));
    EXPECT_EQ(list.size(), 2);
    EXPECT_EQ(list1.size(), 3);

    EXPECT_EQ(list1.at(1), 10);
    EXPECT_EQ(list1.at(2), 20);
    EXPECT_EQ(list1.at(3), 30);

    EXPECT_EQ(list.at(4), 40);
    EXPECT_EQ(list.at(5), 50);
}

TEST(Skiplist, OperatorEq) {
    SkipList<const int, int> list;
    list.insert({ 1, 1 });
    list.insert({ 2, 2 });
    list.insert({ 3, 3 });
    EXPECT_EQ(list.size(), 3);

    SkipList<const int, int> copy = list;
    EXPECT_EQ(copy, list);
    list.insert({ 4, 4 });
    EXPECT_TRUE(copy != list);
}

TEST(Insert, Many) {
    SkipList<int, int> l1;
    for (int i = 0; i < const_size; i++) {
        l1.insert({ i, i });
    }
    EXPECT_EQ(l1.size(), const_size);
}

TEST(Skiplist, InsertDelete) {
    SkipList<const char, int> l1;
    l1.insert({ 'A', 100 });
    EXPECT_EQ(l1.at('A'), 100);
    EXPECT_EQ(l1.size(), 1);
    l1.erase('A');
    EXPECT_ANY_THROW(l1.at('A'));
    EXPECT_EQ(l1.size(), 0);
}

TEST(Erase, Key) {
    SkipList<int, int> l1;
    for (int i = 0; i < const_size; i++) {
        l1.insert({ i, i });
    }

    for (int i = 0; i != const_size; i++) {
        EXPECT_EQ(i, l1.at(i));
    }

    EXPECT_EQ(l1.size(), const_size);
    l1.erase(8);
    EXPECT_EQ(l1.size(), const_size - 1);
    std::cout << "2 " << l1;
}

TEST(Erase, FewKey) {
    SkipList<int, int> l1;
    l1.insert({ 1, 10 });
    l1.insert({ 2, 20 });
    l1.insert({ 3, 30 });
    EXPECT_EQ(l1.at(1), 10);
    EXPECT_EQ(l1.at(2), 20);
    EXPECT_EQ(l1.at(3), 30);
    EXPECT_EQ(l1.size(), 3);
    EXPECT_NO_THROW(l1.erase(2));
    EXPECT_ANY_THROW(l1.at(2));
    EXPECT_EQ(l1.size(), 2);
    EXPECT_NO_THROW(l1.erase(1));
    EXPECT_ANY_THROW(l1.at(1));
    EXPECT_EQ(l1.size(), 1);
    EXPECT_NO_THROW(l1.erase(3));
    EXPECT_ANY_THROW(l1.at(3));
    EXPECT_EQ(l1.size(), 0);
}

TEST(Erase, ManyIterator) {
    SkipList<int, int> l1;
    for (int i = 0; i < const_size; i++) {
        l1.insert({ i, i });
    }

    for (int i = 0; i < const_size; i++) {
        EXPECT_EQ(i, l1.at(i));
    }

    EXPECT_EQ(l1.size(), const_size);
    std::cout << "1 " << l1;
    for (int i = 0; i < const_size; i++) {
        if (i % 2 == 0) {
            EXPECT_NO_THROW(l1.erase(l1.find(i)));
        }
    }
    EXPECT_EQ(l1.size(), const_size / 2);
    EXPECT_ANY_THROW(l1.at(0));
    EXPECT_ANY_THROW(l1.at(4));
    EXPECT_ANY_THROW(l1.at(6));
    std::cout << "2 " << l1;
}

TEST(Erase, Range) {
    SkipList<int, int> l1;
    for (int i = 0; i < const_size; i++) {
        l1.insert({ i, i });
    }
    EXPECT_NO_THROW(l1.erase(l1.find(4), l1.find(10)));
    EXPECT_EQ(l1.size(), const_size - 6);
    EXPECT_NO_THROW(l1.at(0));
    EXPECT_ANY_THROW(l1.at(4));
    EXPECT_ANY_THROW(l1.at(6));
    std::cout << "2 " << l1;
}

TEST(Erase, IteratorThrow) {
    SkipList<int, int> l1;
    EXPECT_ANY_THROW(l1.erase(l1.end()));
}

TEST(Erase, RangeThrow) {
    SkipList<int, int> l1;
    EXPECT_ANY_THROW(l1.erase(l1.begin(), l1.end()));
}

TEST(Skiplist, Clear) {
    SkipList<const int, int> l1;
    for (int i = 0; i < const_size; i++) {
        l1.insert({ i, i });
    }
    EXPECT_EQ(l1.size(), const_size);
    l1.clear();
    EXPECT_EQ(l1.size(), 0);
    EXPECT_EQ(l1.begin(), l1.end());
}

TEST(Skiplist, Empty) {
    SkipList<const int, int> l1;
    EXPECT_TRUE(l1.empty());
    for (int i = 0; i < const_size; i++) {
        l1.insert({ i, i });
    }
    EXPECT_FALSE(l1.empty());
    l1.clear();
    EXPECT_EQ(l1.size(), 0);
    EXPECT_TRUE(l1.empty());
}

TEST(SkiplistIterator, BeginEnd) {
    SkipList<const char, int> l1;
    auto begin1 = l1.begin();
    auto end = l1.end();
    EXPECT_EQ(begin1, end);
}

TEST(Find, CompareIterators) {
    SkipList<const char, int> l1;
    l1.insert({ 'A', 100 });
    l1.insert({ 'B', 200 });
    auto it_1 = l1.find('A');
    auto it_2 = l1.find('B');
    SkipList<const char, int>::iterator it_3;
    it_3 = it_1;
    EXPECT_EQ(it_3->first, 'A');
    EXPECT_EQ(it_3->second, 100);
    it_3 = it_2;
    EXPECT_EQ(it_3->first, 'B');
    EXPECT_EQ(it_3->second, 200);
}

TEST(SkiplistIterator, OperatorEq) {
    SkipList<const char, int> l1;
    l1.insert({ 'A', 1 });
    auto begin1 = l1.begin();
    auto begin2 = l1.begin();
    EXPECT_EQ(begin1, begin2);
}

TEST(Skiplist, OperatorBrackets) {
    SkipList<const char, int> l1;
    l1['A'] = 100;
    EXPECT_EQ(l1.at('A'), 100);
    EXPECT_EQ(l1.at('A'), l1['A']);
}
