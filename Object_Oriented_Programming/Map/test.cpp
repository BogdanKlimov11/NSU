#include <gtest/gtest.h>

#include "map.hpp"

TEST(Insert, Invalid) {
    Map map;
    ASSERT_EQ(map.size(), 0);
    EXPECT_ANY_THROW(map.insert(nullptr));
    EXPECT_ANY_THROW(map.insert(""));
    EXPECT_EQ(map.size(), 0);
}

TEST(Insert, OneKey) {
    Map map;
    ASSERT_EQ(map.size(), 0);
    EXPECT_NO_THROW(map.insert("key"));
    ASSERT_EQ(map.size(), 1);
}

TEST(Insert, OneKeyDouble) {
    Map map;
    ASSERT_EQ(map.size(), 0);
    EXPECT_NO_THROW(map.insert("key"));
    EXPECT_NO_THROW(map.insert("key"));
    ASSERT_EQ(map.size(), 1);
}

TEST(Insert, ManyKey) {
    Map map;
    ASSERT_EQ(map.size(), 0);
    EXPECT_NO_THROW(map.insert("key1"));
    EXPECT_NO_THROW(map.insert("key2"));
    EXPECT_NO_THROW(map.insert("key3"));
    ASSERT_EQ(map.size(), 3);
}

TEST(MapCopy, CopyNull) {
    Map map;
    ASSERT_EQ(map.size(), 0);
    Map map_copy(map);
    ASSERT_EQ(map_copy.size(), 0);
    EXPECT_EQ(map_copy.size(), map.size());
}

TEST(MapCopy, OneKey) {
    Map map;
    ASSERT_EQ(map.size(), 0);
    EXPECT_NO_THROW(map.insert("key1"));
    ASSERT_EQ(map.size(), 1);
    Map map_copy(map);
    ASSERT_EQ(map_copy.size(), 1);
    EXPECT_EQ(map_copy.size(), map.size());
}

TEST(MapCopy, ManyKey) {
    Map map;
    ASSERT_EQ(map.size(), 0);
    EXPECT_NO_THROW(map.insert("key1"));
    EXPECT_NO_THROW(map.insert("key2"));
    EXPECT_NO_THROW(map.insert("key3"));
    ASSERT_EQ(map.size(), 3);
    Map map_copy(map);
    ASSERT_EQ(map_copy.size(), 3);
    EXPECT_EQ(map_copy.size(), map.size());
}

TEST(AssignmentOperator, Null) {
    Map map;
    ASSERT_EQ(map.size(), 0);
    Map map_equal = map;
    EXPECT_EQ(map_equal.size(), 0);
    EXPECT_EQ(map_equal.size(), map.size());
}

TEST(AssignmentOperator, ManyKey) {
    Map map;
    ASSERT_EQ(map.size(), 0);
    EXPECT_NO_THROW(map.insert("key1"));
    EXPECT_NO_THROW(map.insert("key2"));
    EXPECT_NO_THROW(map.insert("key3"));
    Map map_equal = map;
    EXPECT_EQ(map_equal.size(), 3);
    EXPECT_EQ(map_equal.size(), map.size());
}

TEST(MapAt, Null) {
    Map map;
    ASSERT_EQ(map.size(), 0);
    EXPECT_EQ(map.at(0), nullptr);
}

TEST(MapAt, AnyKey) {
    Map map;
    ASSERT_EQ(map.size(), 0);
    int& value = map.insert("KEY");
    value = 79;
    EXPECT_EQ(map.at(0)->value, 79);
    EXPECT_EQ(map.at(1), nullptr);
}

TEST(MapFind, AnyKey) {
    Map map;
    int& value = map.insert("key1");
    value = 10;
    EXPECT_EQ(map.at(0)->value, 10);
    EXPECT_NO_THROW(map.insert("key2"));
    EXPECT_NO_THROW(map.find("key2"));
    EXPECT_NO_THROW(map.find("key1"));
    EXPECT_EQ(map.find("kEyYY"), nullptr);
}

TEST(MapErase, ThreeKeys) {
    Map map;
    int& value0 = map.insert("key1");
    value0 = 21;
    int& value1 = map.insert("key2");
    value1 = 22;
    int& value2 = map.insert("key3");
    value2 = 23;
    EXPECT_EQ(map.size(), 3);
    map.erase("key1");
    EXPECT_EQ(map.size(), 2);
    EXPECT_EQ(map.find("key1"), nullptr);
    map.erase("key2");
    EXPECT_EQ(map.size(), 1);
}

TEST(Brackets, MapBrackets) {
    Map map;
    EXPECT_NO_THROW(map.insert("key1"));
    EXPECT_NO_THROW(map["key2"] = 7);
    EXPECT_EQ(map.size(), 2);
    EXPECT_EQ(map["key2"], 7);
}

TEST(Brackets, EqualKeys) {
    Map map;
    EXPECT_ANY_THROW(map[""] = 7);
    EXPECT_NO_THROW(map.insert("key1"));
    EXPECT_NO_THROW(map.insert("key2"));
    EXPECT_NO_THROW(map.insert("key3"));
    EXPECT_EQ(map.size(), 3);
    EXPECT_NO_THROW(map["key1"]);
    Map map_copy(map);
    EXPECT_NO_THROW(map_copy["key2"]);
    EXPECT_EQ(map.size(), 3);
}

TEST(ConstBrackets, EqualValue) {
    Map map;
    EXPECT_NO_THROW(map.insert("key1"));
    EXPECT_NO_THROW(map["key8"] = 8);
    EXPECT_EQ(map.size(), 2);
    EXPECT_EQ(map["key8"], 8);
    EXPECT_NO_THROW(map["key2"] = 2);
    EXPECT_EQ(map.size(), 3);
    EXPECT_EQ(map["key2"], 2);
}

TEST(ConstBrackets, Invalid) {
    Map map;
    EXPECT_EQ(map.size(), 0);
    EXPECT_NO_THROW(map.insert("key1"));
    const Map map_copy(map);
    EXPECT_ANY_THROW(map_copy[""]);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
