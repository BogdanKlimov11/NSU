#ifndef LAB_SET_TEST_UTILITIES_H
#define LAB_SET_TEST_UTILITIES_H

#include <stdbool.h>

typedef struct {
    size_t *arr;
    size_t length;
} testItem;

size_t testHash(const testItem *item);
bool testEquals(const testItem *left, const testItem *right);
bool testDestroy1(testItem *item);
bool testDestroy2(testItem *item);
testItem createItem1(size_t size);
testItem *createItem2(size_t size);

#endif //LAB_SET_TEST_UTILITIES_H
