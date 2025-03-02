#include <gtest/gtest.h>

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

TEST(BloomFilterC, NewFree) {
    BloomFilter* filter = bloom_filter_new(128, string_hash, 1);
    ASSERT_TRUE(filter != nullptr);
    bloom_filter_free(filter);
    filter = bloom_filter_new(128, string_hash, 64);
    ASSERT_TRUE(filter != nullptr);
    bloom_filter_free(filter);
    filter = bloom_filter_new(128, string_hash, 50000);
    ASSERT_TRUE(filter == nullptr);
}

TEST(BloomFilterC, InsertQuery) {
    BloomFilter* filter = bloom_filter_new(128, string_hash, 4);
    EXPECT_EQ(bloom_filter_query(filter, static_cast<BloomFilterValue>("test 1")), 0);
    EXPECT_EQ(bloom_filter_query(filter, static_cast<BloomFilterValue>("test 2")), 0);
    bloom_filter_insert(filter, static_cast<BloomFilterValue>("test 1"));
    bloom_filter_insert(filter, static_cast<BloomFilterValue>("test 2"));
    EXPECT_NE(bloom_filter_query(filter, static_cast<BloomFilterValue>("test 1")), 0);
    EXPECT_NE(bloom_filter_query(filter, static_cast<BloomFilterValue>("test 2")), 0);
    bloom_filter_free(filter);
}

TEST(BloomFilterC, ReadLoad) {
    BloomFilter* filter1 = bloom_filter_new(128, string_hash, 4);
    bloom_filter_insert(filter1, static_cast<BloomFilterValue>("test 1"));
    bloom_filter_insert(filter1, static_cast<BloomFilterValue>("test 2"));
    unsigned char state[16];
    bloom_filter_read(filter1, state);
    bloom_filter_free(filter1);

    BloomFilter* filter2 = bloom_filter_new(128, string_hash, 4);
    bloom_filter_load(filter2, state);
    EXPECT_NE(bloom_filter_query(filter2, static_cast<BloomFilterValue>("test 1")), 0);
    EXPECT_NE(bloom_filter_query(filter2, static_cast<BloomFilterValue>("test 2")), 0);
    bloom_filter_free(filter2);
}

TEST(BloomFilterC, Intersection) {
    BloomFilter* filter1 = bloom_filter_new(128, string_hash, 4);
    bloom_filter_insert(filter1, static_cast<BloomFilterValue>("test 1"));
    bloom_filter_insert(filter1, static_cast<BloomFilterValue>("test 2"));

    BloomFilter* filter2 = bloom_filter_new(128, string_hash, 4);
    bloom_filter_insert(filter2, static_cast<BloomFilterValue>("test 1"));
    EXPECT_EQ(bloom_filter_query(filter2, static_cast<BloomFilterValue>("test 2")), 0);

    BloomFilter* result = bloom_filter_intersection(filter1, filter2);
    EXPECT_NE(bloom_filter_query(result, static_cast<BloomFilterValue>("test 1")), 0);
    assert(bloom_filter_query(result, static_cast<BloomFilterValue>("test 2")) == 0);

    bloom_filter_free(result);
    bloom_filter_free(filter1);
    bloom_filter_free(filter2);
}

TEST(BloomFilterC, Union) {
    BloomFilter* filter1 = bloom_filter_new(128, string_hash, 4);
    bloom_filter_insert(filter1, static_cast<BloomFilterValue>("test 1"));

    BloomFilter* filter2 = bloom_filter_new(128, string_hash, 4);
    bloom_filter_insert(filter2, static_cast<BloomFilterValue>("test 2"));

    BloomFilter* result = bloom_filter_union(filter1, filter2);
    EXPECT_NE(bloom_filter_query(result, static_cast<BloomFilterValue>("test 1")), 0);
    EXPECT_NE(bloom_filter_query(result, static_cast<BloomFilterValue>("test 2")), 0);

    bloom_filter_free(result);
    bloom_filter_free(filter1);
    bloom_filter_free(filter2);
}
