#include <stdlib.h>

#include "test.h"

size_t testHash(const testItem *item) {
    size_t hash = 0;
    for (size_t i = 0; i < item->length; i++) {
        hash += item->arr[i];
    }
    return hash;
}

bool testEquals(const testItem *left, const testItem *right) {
    if (left->length != right->length) { return false; }
    for (size_t i = 0; i < left->length; i++) {
        if (left->arr[i] != right->arr[i]) { return false; }
    }
    return true;
}

bool testDestroy1(testItem *item) {
    free(item->arr);
    return true;
}

bool testDestroy2(testItem *item) {
    free(item->arr);
    item = NULL;
    return true;
}

testItem createItem1(size_t size) {
    testItem item;
    item.arr = (size_t *) calloc(size, sizeof(size_t));
    item.length = size;
    return item;
}

testItem *createItem2(size_t size) {
    testItem *item = (testItem *) calloc(1, sizeof(testItem));
    item->arr = (size_t *) calloc(size, sizeof(size_t));
    item->length = size;
    return item;
}
