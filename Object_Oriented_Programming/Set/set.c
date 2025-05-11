#include <stdlib.h>
#include <stdbool.h>
#include <vcruntime_string.h>

#include "set.h"

enum bucket_state {
    FREE,
    FULL,
    DELETED
};

typedef struct {
    void *items;
    int *conditions;
    size_t capacity;
    size_t itemSize;
    size_t itemsAmount;
    size_t (*hash)(const void *);
    bool (*equals)(const void *, const void *);
} pSet;

static pSet *createSetCopy(pSet *originSet) {
    pSet *setCopy = (pSet *) malloc(sizeof(pSet));
    if (setCopy == NULL)
        return NULL;
    setCopy->items = originSet->items;
    setCopy->conditions = originSet->conditions;
    setCopy->capacity = originSet->capacity;
    setCopy->itemSize = originSet->itemSize;
    setCopy->itemsAmount = originSet->itemsAmount;
    setCopy->hash = originSet->hash;
    setCopy->equals = originSet->equals;
    return setCopy;
}

static bool setResize(pSet *originSet, size_t itemsAmount, size_t newSize) {
    pSet *newSet = createSetCopy(originSet);
    int *newConditions = (int *) malloc(sizeof(int) * newSize);
    if (newConditions == NULL)
        return NULL;
    for (size_t i = 0; i < newSize; i++) {
        newConditions[i] = FREE;
    }
    void *newItems = (void *) malloc(newSize * originSet->itemSize);
    if (newItems == NULL || newConditions == NULL || newSet == NULL) {
        free(newSet);
        free(newItems);
        free(newConditions);
        return false;
    }
    originSet->items = newItems;
    originSet->conditions = newConditions;
    originSet->capacity = newSize;
    originSet->itemsAmount = 0;
    const void *current;
    for (size_t i = 0; i < itemsAmount; i++) {
        current = set_current(newSet, i);
        set_insert(originSet, current);
    }
    free(newSet->conditions);
    free(newSet->items);
    free(newSet);
    return true;
}

static bool copyItem(const void *source, void *copy, size_t itemSize) {
    if (source == NULL || copy == NULL)
        return false;
    memcpy(copy, source, itemSize);
    return true;
}

static size_t getIndex(pSet *set, size_t itemId) {
    if (itemId + 1 == 0)
        return set->capacity - 1;
    return itemId % set->capacity;
}

void *set_create(size_t itemSize, size_t hash(const void *), bool(*equals)(const void *, const void *)) {
    if (hash == NULL || equals == NULL || itemSize == 0) {
        return NULL;
    }
    pSet *setPtr = (pSet *) malloc(sizeof(pSet));
    if (setPtr == NULL) {
        free(setPtr);
        return NULL;
    }
    setPtr->capacity = 5;
    int *newConditions = (int *) malloc(sizeof(int) * setPtr->capacity);
    if (newConditions == NULL)
        return NULL;
    for (size_t i = 0; i < setPtr->capacity; i++) {
        newConditions[i] = FREE;
    }
    setPtr->conditions = newConditions;
    setPtr->items = NULL;
    setPtr = set_init(setPtr, itemSize, hash, equals, NULL);
    if (setPtr->items == NULL || setPtr->conditions == NULL) {
        set_destroy(setPtr, NULL);
        return NULL;
    }
    return setPtr;
}

void set_destroy(void *set, void(*destroy)(void *)) {
    if (set == NULL)
        return;
    pSet *setPtr = (pSet *) set;
    set_clear(set, destroy);
    free(setPtr->items);
    free(setPtr->conditions);
    free(set);
}

void *set_init(void *set, size_t itemSize, size_t hash(const void *), bool(*equals)(const void *, const void *),
               void(*destroy)(void *)) {
    if (set == NULL || hash == NULL || equals == NULL || itemSize == 0)
        return NULL;
    set_clear(set, destroy);
    pSet *setPtr = (pSet *) set;
    if (setPtr->items != NULL)
        free(setPtr->items);
    setPtr->items = (void *) malloc(setPtr->capacity * itemSize);
    if (setPtr->items == NULL) {
        set_destroy(set, destroy);
        return NULL;
    }
    setPtr->itemSize = itemSize;
    setPtr->hash = hash;
    setPtr->equals = equals;
    setPtr->itemsAmount = 0;
    return set;
}

void set_clear(void *set, void(*destroy)(void *)) {
    if (set == NULL)
        return;
    pSet *setPtr = (pSet *) (set);
    for (size_t i = 0; i < setPtr->capacity; i++) {
        set_erase(set, i, destroy);
    }
}

size_t set_count(const void *set) {
    if (set == NULL)
        return INVALID;
    pSet *setPtr = (pSet *) set;
    return setPtr->itemsAmount;
}

bool set_contains(const void *set, const void *item) {
    if (set == NULL || item == NULL)
        return false;
    pSet *setPtr = (pSet *) set;
    size_t index = getIndex(setPtr, setPtr->hash(item));
    for (size_t i = index; i != getIndex(setPtr, index - 1); i = getIndex(setPtr, i + 1)) {
        if (setPtr->conditions[i] == FREE)
            return false;
        if (setPtr->conditions[i] == FULL && setPtr->equals(item, set_current(set, i)))
            return true;
    }
    return false;
}

bool set_insert(void *set, const void *item) {
    if (set == NULL || item == NULL) { return false; }
    pSet *setPtr = (pSet *) set;
    if (setPtr->itemsAmount + 1 == setPtr->capacity) {
        bool isResized = setResize(set, setPtr->capacity, setPtr->capacity * 2);
        if (!isResized)
            return false;
    }
    size_t index = getIndex(setPtr, setPtr->hash(item));
    const void *current;
    for (size_t i = index; i != getIndex(setPtr, index - 1); i = getIndex(setPtr, i + 1)) {
        current = set_current(set, i);
        if (current != NULL && setPtr->conditions[i] == FULL && setPtr->equals(item, current)) {
            return false;
        }
        if (setPtr->conditions[i] == FREE || setPtr->conditions[i] == DELETED) {
            bool isCopied = copyItem(item, (char *) setPtr->items + i * setPtr->itemSize, setPtr->itemSize);
            if (!isCopied)
                return false;
            setPtr->conditions[i] = FULL;
            setPtr->itemsAmount++;
            return true;
        }
    }
    return false;
}

void set_remove(void *set, const void *item, void(*destroy)(void *)) {
    if (set == NULL || item == NULL) { return; }
    pSet *setPtr = (pSet *) set;
    size_t index = getIndex(setPtr, setPtr->hash(item));
    const void *current;
    for (size_t i = index; i != getIndex(setPtr, index - 1); i = getIndex(setPtr, i + 1)) {
        current = set_current(set, i);
        if (current != NULL && setPtr->equals(item, current)) {
            set_erase(set, i, destroy);
            return;
        }
        if (setPtr->conditions[i] == FREE)
            return;
    }
}

size_t set_first(const void *set) {
    if (set == NULL)
        return set_stop(set);
    pSet *setPtr = (pSet *) set;
    if (setPtr->itemsAmount == 0)
        return set_stop(set);
    for (size_t i = 0; i < setPtr->capacity; i++) {
        if (setPtr->conditions[i] == FULL)
            return i;
    }
    return set_stop(set);
}

size_t set_last(const void *set) {
    if (set == NULL)
        return set_stop(set);
    pSet *setPtr = (pSet *) set;
    if (setPtr->itemsAmount == 0)
        return set_stop(set);
    for (size_t i = setPtr->capacity - 1; i >= 0; i--) {
        if (setPtr->conditions[i] == FULL)
            return i;
    }
    return set_stop(set);
}

size_t set_next(const void *set, size_t item_id) {
    if (set == NULL)
        return set_stop(set);
    pSet *setPtr = (pSet *) set;
    if (item_id + 1 >= setPtr->capacity || setPtr->conditions[item_id] != FULL ||
        setPtr->itemsAmount == 0)
        return set_stop(set);
    for (size_t i = item_id + 1; i < setPtr->capacity; i++) {
        if (setPtr->conditions[i] == FULL)
            return i;
    }
    return set_stop(set);
}

size_t set_prev(const void *set, size_t item_id) {
    if (set == NULL)
        return set_stop(set);
    pSet *setPtr = (pSet *) set;
    if (item_id >= setPtr->capacity || setPtr->conditions[item_id] != FULL || setPtr->itemsAmount == 0 ||
        item_id == 0)
        return set_stop(set);
    for (size_t i = item_id - 1;; i--) {
        if (setPtr->conditions[i] == FULL)
            return i;
        if (i == 0)
            break;
    }
    return set_stop(set);
}

size_t set_stop(const void *set) {
    return INVALID - 1;
}

const void *set_current(const void *set, size_t item_id) {
    if (set == NULL)
        return NULL;
    pSet *setPtr = (pSet *) set;
    if (item_id >= setPtr->capacity)
        return NULL;
    if (setPtr->conditions[item_id] != FULL)
        return NULL;
    return (char *) setPtr->items + setPtr->itemSize * item_id;
}

void set_erase(void *set, size_t item_id, void(*destroy)(void *)) {
    if (set == NULL)
        return;
    pSet *setPtr = (pSet *) set;
    if (item_id >= setPtr->capacity)
        return;
    if (setPtr->conditions[item_id] == FULL) {
        if(destroy != NULL)
            destroy((char *) setPtr->items + setPtr->itemSize * item_id);
        setPtr->conditions[item_id] = DELETED;
        setPtr->itemsAmount--;
    }
}
