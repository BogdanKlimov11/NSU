#include <gtest/gtest.h>

#include <functional>
#include <bitset>
#include <string>

#include "bloom_filter.h"
extern "C" {
#include "refactoring.h"
}

unsigned int string_hash(void* string) {
    unsigned int result = 5381;
    unsigned char* p = static_cast<unsigned char*>(string);
    while (*p != '\0') {
        result = (result << 5) + result + *p;
        ++p;
    }
    return result;
}

unsigned int double_hash(BloomFilterValue value) {
    unsigned int result = 5381;
    unsigned char* p = static_cast<unsigned char*>(value);
    while (*p != '\0') {
        result = (result << 5) + result + *p;
        ++p;
    }
    return result;
}

unsigned int int_hash(BloomFilterValue value) {
    unsigned int result = 5381;
    unsigned char* p = static_cast<unsigned char*>(value);
    while (*p != '\0') {
        result = (result << 5) + result + *p;
        ++p;
    }
    return result;
}

BloomFilterCpp<int, std::hash<int>, 3, 4> bf;
BloomFilterCpp<double> basic_bf;

TEST(BloomFilterCpp, TableSize) {
    EXPECT_EQ(bf.table_size(), 3);
    EXPECT_EQ(basic_bf.table_size(), 50);
}

TEST(BloomFilterCpp, NumOfFunctions) {
    EXPECT_EQ(bf.num_of_functions(), 4);
    EXPECT_EQ(basic_bf.num_of_functions(), 64);
}

TEST(BloomFilterCpp, Table) {
    EXPECT_EQ(bf.read(), std::bitset<3>{});
    EXPECT_EQ(basic_bf.read(), std::bitset<50>{});
}

TEST(BloomFilterCpp, InsertQueryTestDouble) {
    BloomFilterCpp<double> double_bf;
    BloomFilter* filter = bloom_filter_new(128, static_cast<BloomFilterHashFunc>(double_hash), 4);
    double a = 34.2;
    double b = 34.3;
    double c = 1;
    EXPECT_FALSE(double_bf.query(34.2));
    EXPECT_FALSE(double_bf.query(34.3));
    EXPECT_FALSE(double_bf.query(1));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&a)));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&b)));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&c)));

    double_bf.insert(34.2);
    bloom_filter_insert(filter, static_cast<BloomFilterValue>(&a));
    EXPECT_TRUE(double_bf.query(34.2));
    EXPECT_FALSE(double_bf.query(34.3));
    EXPECT_FALSE(double_bf.query(1));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&a)));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&b)));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&c)));

    double_bf.insert(34.3);
    bloom_filter_insert(filter, static_cast<BloomFilterValue>(&b));
    EXPECT_TRUE(double_bf.query(34.2));
    EXPECT_TRUE(double_bf.query(34.3));
    EXPECT_FALSE(double_bf.query(1));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&a)));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&b)));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&c)));

    double_bf.insert(1);
    bloom_filter_insert(filter, static_cast<BloomFilterValue>(&c));
    EXPECT_TRUE(double_bf.query(34.2));
    EXPECT_TRUE(double_bf.query(34.3));
    EXPECT_TRUE(double_bf.query(1));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&a)));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&b)));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&c)));
    bloom_filter_free(filter);
}

TEST(BloomFilterCpp, InsertQueryTestString) {
    BloomFilterCpp<std::string> string_bf;
    BloomFilter* filter = bloom_filter_new(128, static_cast<BloomFilterHashFunc>(string_hash), 4);
    EXPECT_FALSE(string_bf.query("a"));
    EXPECT_FALSE(string_bf.query("b"));
    EXPECT_FALSE(string_bf.query("c"));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>("a")));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>("b")));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>("c")));

    string_bf.insert("a");
    bloom_filter_insert(filter, static_cast<BloomFilterValue>("a"));
    EXPECT_TRUE(string_bf.query("a"));
    EXPECT_FALSE(string_bf.query("b"));
    EXPECT_FALSE(string_bf.query("c"));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>("a")));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>("b")));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>("c")));

    string_bf.insert("b");
    bloom_filter_insert(filter, static_cast<BloomFilterValue>("b"));
    EXPECT_TRUE(string_bf.query("a"));
    EXPECT_TRUE(string_bf.query("b"));
    EXPECT_FALSE(string_bf.query("c"));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>("a")));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>("b")));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>("c")));

    string_bf.insert("c");
    bloom_filter_insert(filter, static_cast<BloomFilterValue>("c"));
    EXPECT_TRUE(string_bf.query("a"));
    EXPECT_TRUE(string_bf.query("b"));
    EXPECT_TRUE(string_bf.query("c"));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>("a")));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>("b")));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>("c")));
    bloom_filter_free(filter);
}

TEST(BloomFilterCpp, InsertQueryTestInt) {
    BloomFilterCpp<int> int_bf;
    BloomFilter* filter = bloom_filter_new(128, static_cast<BloomFilterHashFunc>(int_hash), 4);
    int a = 1;
    int b = 2;
    int c = 3;
    EXPECT_FALSE(int_bf.query(1));
    EXPECT_FALSE(int_bf.query(2));
    EXPECT_FALSE(int_bf.query(3));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&a)));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&b)));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&c)));

    int_bf.insert(1);
    bloom_filter_insert(filter, static_cast<BloomFilterValue>(&a));
    EXPECT_TRUE(int_bf.query(1));
    EXPECT_FALSE(int_bf.query(2));
    EXPECT_FALSE(int_bf.query(3));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&a)));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&b)));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&c)));

    int_bf.insert(2);
    bloom_filter_insert(filter, static_cast<BloomFilterValue>(&b));
    EXPECT_TRUE(int_bf.query(1));
    EXPECT_TRUE(int_bf.query(2));
    EXPECT_FALSE(int_bf.query(3));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&a)));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&b)));
    EXPECT_FALSE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&c)));

    int_bf.insert(3);
    bloom_filter_insert(filter, static_cast<BloomFilterValue>(&c));
    EXPECT_TRUE(int_bf.query(1));
    EXPECT_TRUE(int_bf.query(2));
    EXPECT_TRUE(int_bf.query(3));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&a)));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&b)));
    EXPECT_TRUE(bloom_filter_query(filter, static_cast<BloomFilterValue>(&c)));
    bloom_filter_free(filter);
}

TEST(BloomFilterCpp, LoadRead) {
    BloomFilterCpp<int> int_bf;
    EXPECT_EQ(int_bf.read(), std::bitset<50>{});
    EXPECT_EQ(bf.read(), std::bitset<3>{});
    EXPECT_EQ(basic_bf.read(), std::bitset<50>{});

    BloomFilter* filter1 = bloom_filter_new(128, int_hash, 4);
    BloomFilter* filter2 = bloom_filter_new(128, int_hash, 4);
    unsigned char state[16];
    bloom_filter_read(filter1, state);

    auto read_bitset = int_bf.read();
    read_bitset.set();
    int_bf.load(read_bitset);
    bloom_filter_load(filter2, state);

    EXPECT_FALSE(int_bf.read() == std::bitset<50>{});
    EXPECT_EQ(int_bf.read(), std::bitset<50>{}.set());
    for (int i = 0; i < 1000; i++) {
        ASSERT_TRUE(int_bf.query(i));
        ASSERT_FALSE(bloom_filter_query(filter2, static_cast<BloomFilterValue>(&i)));
    }
    bloom_filter_free(filter1);
    bloom_filter_free(filter2);
}

TEST(BloomFilterCpp, Union) {
    BloomFilterCpp<int, std::hash<int>, 3, 3> int_bf1;
    BloomFilterCpp<int, std::hash<int>, 3, 3> int_bf2;
    BloomFilter* filter1 = bloom_filter_new(128, string_hash, 4);
    BloomFilter* filter2 = bloom_filter_new(128, string_hash, 4);
    unsigned char state[16];
    BloomFilter* result1 = bloom_filter_union(filter1, filter2);
    auto empty_union = int_bf1 | int_bf2;
    EXPECT_EQ(int_bf1.read(), std::bitset<3>{});
    EXPECT_EQ(int_bf2.read(), std::bitset<3>{});
    bloom_filter_read(result1, state);
    EXPECT_EQ(empty_union.read(), std::bitset<3>{});
    EXPECT_EQ(empty_union.read(), 0);

    int a = 1;
    int b = 1;
    bloom_filter_insert(filter1, static_cast<BloomFilterValue>(&a));
    bloom_filter_insert(filter1, static_cast<BloomFilterValue>(&b));
    int_bf1.insert(1);
    int_bf2.insert(2);
    auto non_empty_union = int_bf1 | int_bf2;
    BloomFilter* result2 = bloom_filter_union(filter1, filter2);
    EXPECT_EQ(non_empty_union.read(), std::bitset<3>{}.set());
    EXPECT_NE(bloom_filter_query(result2, static_cast<BloomFilterValue>(&a)), 0);
    EXPECT_NE(bloom_filter_query(result2, static_cast<BloomFilterValue>(&b)), 0);

    bloom_filter_free(result2);
    bloom_filter_free(result1);
    bloom_filter_free(filter1);
    bloom_filter_free(filter2);
}

TEST(BloomFilterCpp, Intersection) {
    BloomFilterCpp<int, std::hash<int>, 3, 3> int_bf1;
    BloomFilterCpp<int, std::hash<int>, 3, 3> int_bf2;
    BloomFilter* filter1 = bloom_filter_new(128, string_hash, 4);
    BloomFilter* filter2 = bloom_filter_new(128, string_hash, 4);
    unsigned char state[16];
    BloomFilter* result1 = bloom_filter_intersection(filter1, filter2);
    auto empty_intersection = int_bf1 & int_bf2;
    EXPECT_EQ(int_bf1.read(), std::bitset<3>{});
    EXPECT_EQ(int_bf2.read(), std::bitset<3>{});
    bloom_filter_read(result1, state);
    EXPECT_EQ(empty_intersection.read(), std::bitset<3>{});
    EXPECT_EQ(empty_intersection.read(), 0);

    int a = 1;
    int b = 1;
    bloom_filter_insert(filter1, static_cast<BloomFilterValue>(&a));
    bloom_filter_insert(filter1, static_cast<BloomFilterValue>(&b));
    int_bf1.insert(1);
    int_bf2.insert(2);
    auto non_empty_intersection = int_bf1 & int_bf2;
    EXPECT_EQ(non_empty_intersection.read(), std::bitset<3>{}.set(2));
    BloomFilter* result2 = bloom_filter_union(filter1, filter2);
    EXPECT_NE(bloom_filter_query(result2, static_cast<BloomFilterValue>(&a)), 0);
    EXPECT_NE(bloom_filter_query(result2, static_cast<BloomFilterValue>(&b)), 0);
    bloom_filter_free(result2);
    bloom_filter_free(result1);
    bloom_filter_free(filter1);
    bloom_filter_free(filter2);
}

TEST(BloomFilterCpp, UnionAssignment) {
    BloomFilterCpp<int, std::hash<int>, 3, 3> int_bf1;
    BloomFilterCpp<int, std::hash<int>, 3, 3> int_bf2;
    int_bf1 |= int_bf2;
    EXPECT_EQ(int_bf1.read(), std::bitset<3>{});
    EXPECT_EQ(int_bf2.read(), std::bitset<3>{});

    int_bf1.insert(1);
    int_bf2.insert(2);
    int_bf1 |= int_bf2;
    EXPECT_EQ(int_bf1.read(), std::bitset<3>{}.set());
}

TEST(BloomFilterCpp, IntersectionAssignment) {
    BloomFilterCpp<int, std::hash<int>, 3, 3> int_bf1;
    BloomFilterCpp<int, std::hash<int>, 3, 3> int_bf2;
    int_bf1 &= int_bf2;
    EXPECT_EQ(int_bf1.read(), std::bitset<3>{});
    EXPECT_EQ(int_bf2.read(), std::bitset<3>{});

    int_bf1.insert(1);
    int_bf2.insert(2);
    int_bf1 &= int_bf2;
}
