#include <gtest/gtest.h>

#include <vector>

#include "shared_ptr.hpp"

class ArrayAssertHelper {
public:
    ArrayAssertHelper() { ++instances; }
    ~ArrayAssertHelper() { --instances; }
    static int instances;
};

int ArrayAssertHelper::instances = 0;

struct TstStruct {
    explicit TstStruct(int value) : m_value(value) { ++instances; }
    ~TstStruct() { --instances; }
    int m_value;
    static int instances;
};

int TstStruct::instances = 0;

TEST(SharedPTR, basic) {
    {
        SharedPTR<TstStruct> testPtr(new TstStruct(10));
        EXPECT_TRUE(testPtr);
        EXPECT_TRUE(testPtr.use_count() == 1);
        EXPECT_TRUE(testPtr.unique());
        EXPECT_FALSE(testPtr.get() == nullptr);
        EXPECT_EQ(testPtr->m_value, 10);
        EXPECT_TRUE(testPtr->instances == TstStruct::instances);
        {
            SharedPTR<TstStruct> anotherPtr(testPtr);
            EXPECT_TRUE(testPtr);
            EXPECT_EQ(testPtr.get(), anotherPtr.get());
            EXPECT_TRUE(anotherPtr);
            EXPECT_FALSE(anotherPtr.unique());
            EXPECT_TRUE(testPtr.use_count() == 2);
            EXPECT_FALSE(testPtr.unique());
            EXPECT_FALSE(testPtr.get() == nullptr);
            EXPECT_TRUE(testPtr->instances == TstStruct::instances);
            EXPECT_TRUE(testPtr->instances == 1);
        }
        EXPECT_TRUE(testPtr.use_count() == 1);
        EXPECT_TRUE(testPtr.unique());
        EXPECT_FALSE(testPtr.get() == nullptr);
        {
            SharedPTR<TstStruct> anotherPtr = testPtr;
            EXPECT_TRUE(testPtr);
            EXPECT_EQ(testPtr.get(), anotherPtr.get());
            EXPECT_TRUE(anotherPtr);
            EXPECT_FALSE(anotherPtr.unique());
            EXPECT_TRUE(testPtr.use_count() == 2);
            EXPECT_FALSE(testPtr.unique());
            EXPECT_FALSE(testPtr.get() == nullptr);
            EXPECT_TRUE(testPtr->instances == TstStruct::instances);
            EXPECT_TRUE(testPtr->instances == 1);
        }
    }
    EXPECT_TRUE(TstStruct::instances == 0);
}

TEST(SharedPTR, reset) {
    SharedPTR<TstStruct> testPtr;
    testPtr.reset(new TstStruct(10));
    EXPECT_TRUE(testPtr);
    EXPECT_TRUE(testPtr.use_count() == 1);
    EXPECT_TRUE(testPtr.unique());
    EXPECT_FALSE(testPtr.get() == nullptr);
    EXPECT_EQ(testPtr->m_value, 10);
    EXPECT_TRUE(testPtr->instances == TstStruct::instances);
    testPtr.reset();
    EXPECT_FALSE(testPtr);
    EXPECT_FALSE(testPtr.unique());
    EXPECT_TRUE(testPtr.use_count() == 0);
    EXPECT_TRUE(testPtr.get() == nullptr);
}

TEST(SharedPTR, release) {
    SharedPTR<TstStruct> testPtr(new TstStruct(10));
    EXPECT_TRUE(testPtr);
    EXPECT_TRUE(testPtr.use_count() == 1);
    EXPECT_TRUE(testPtr.unique());
    EXPECT_FALSE(testPtr.get() == nullptr);
    EXPECT_EQ(testPtr->m_value, 10);
    EXPECT_TRUE(testPtr->instances == TstStruct::instances);
    testPtr.release();
    EXPECT_FALSE(testPtr);
    EXPECT_FALSE(testPtr.unique());
    EXPECT_TRUE(testPtr.use_count() == 0);
    EXPECT_TRUE(testPtr.get() == nullptr);
}

TEST(SharedPTR, array) {
    {
        SharedPTR<ArrayAssertHelper[]> testPtr(new ArrayAssertHelper[10]);
        EXPECT_TRUE(testPtr);
        EXPECT_EQ(ArrayAssertHelper::instances, 10);
        EXPECT_TRUE(testPtr.use_count() == 1);
        EXPECT_TRUE(testPtr.unique());
        EXPECT_FALSE(testPtr.get() == nullptr);
    }
    EXPECT_EQ(ArrayAssertHelper::instances, 0);
}

TEST(SharedPTR, swap) {
    SharedPTR<TstStruct> testPtr(new TstStruct(10));
    EXPECT_TRUE(testPtr);
    EXPECT_TRUE(testPtr.use_count() == 1);
    EXPECT_TRUE(testPtr.unique());
    EXPECT_EQ(testPtr->m_value, 10);
    SharedPTR<TstStruct> anotherPtr(new TstStruct(11));
    EXPECT_TRUE(anotherPtr);
    EXPECT_TRUE(anotherPtr.use_count() == 1);
    EXPECT_TRUE(anotherPtr.unique());
    EXPECT_EQ(anotherPtr->m_value, 11);
    testPtr.swap(anotherPtr);
    EXPECT_EQ(testPtr->m_value, 11);
    EXPECT_EQ(anotherPtr->m_value, 10);
    EXPECT_TRUE(testPtr.use_count() == 1);
    EXPECT_TRUE(testPtr.unique());
    EXPECT_TRUE(anotherPtr.use_count() == 1);
    EXPECT_TRUE(anotherPtr.unique());
}

TEST(SharedPTR, empty) {
    SharedPTR<TstStruct> testPtr;
    EXPECT_FALSE(testPtr);
    EXPECT_FALSE(testPtr.unique());
    EXPECT_TRUE(testPtr.use_count() == 0);
    EXPECT_TRUE(testPtr.get() == nullptr);
}

TEST(SharedPTR, assigment) {
    {
        SharedPTR<TstStruct> testPtr(new TstStruct(10));
        EXPECT_TRUE(testPtr);
        EXPECT_TRUE(testPtr.use_count() == 1);
        EXPECT_TRUE(testPtr.unique());
        EXPECT_FALSE(testPtr.get() == nullptr);
        testPtr = testPtr;
        EXPECT_TRUE(testPtr.use_count() == 1);
        EXPECT_TRUE(testPtr.unique());
        EXPECT_FALSE(testPtr.get() == nullptr);
    }
    {
        auto ptr = new TstStruct(10);
        SharedPTR<TstStruct> testPtr;
        testPtr = ptr;
        EXPECT_TRUE(testPtr);
        EXPECT_TRUE(testPtr.use_count() == 1);
        EXPECT_TRUE(testPtr.unique());
        EXPECT_FALSE(testPtr.get() == nullptr);
    }
    {
        SharedPTR<TstStruct> testPtr(new TstStruct(10));
        EXPECT_TRUE(testPtr);
        EXPECT_TRUE(testPtr.use_count() == 1);
        EXPECT_TRUE(testPtr.unique());
        EXPECT_FALSE(testPtr.get() == nullptr);
        testPtr = std::move(testPtr);
        EXPECT_TRUE(testPtr.use_count() == 1);
        EXPECT_TRUE(testPtr.unique());
        EXPECT_FALSE(testPtr.get() == nullptr);
    }
}

TEST(SharedPTR, MakeShared) {
    auto testPtr = makeShared<TstStruct>(10);
    EXPECT_TRUE(testPtr);
    EXPECT_TRUE(testPtr.use_count() == 1);
    EXPECT_TRUE(testPtr.unique());
    EXPECT_FALSE(testPtr.get() == nullptr);
}
