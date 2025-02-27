#include <stdlib.h>
#include <crtdbg.h>
#include <stddef.h>
#include <assert.h>

#include "set.h"
#include "test.h"

int main() {
    ///TEST 1///
    {
        testItem *item1 = createItem2(100);
        testItem *item2 = createItem2(100);
        void *set = set_create(sizeof(testItem), testHash, testEquals);
        set = set_init(set, sizeof(testItem), testHash, testEquals, testDestroy2);
        set_insert(set, item1);
        assert(set_count(set) == 1);
        assert(set_contains(set, item1));
        set_remove(set, item1, testDestroy2);
        assert(!set_contains(set, item1));
        assert(set_count(set) == 0);
        set_insert(set, item2);
        assert(set_count(set) == 1);
        assert(set_contains(set, item2));
        set_destroy(set, testDestroy2);
        free(item1);
        free(item2);
    }

    ///TEST 2///
    {
        testItem item1 = createItem1(100);
        void *set = set_create(sizeof(testItem), testHash, testEquals);
        set = set_init(set, sizeof(testItem), testHash, testEquals, testDestroy1);
        size_t first_index, stop_index;
        set_insert(set, &item1);
        stop_index = set_stop(set);
        first_index = set_first(set);
        assert(stop_index == set_next(set, first_index));
        set_destroy(set, testDestroy1);
    }

    ///TEST 3///
    {
        testItem item1 = createItem1(100);
        void *set = set_create(sizeof(testItem), testHash, testEquals);
        set = set_init(set, sizeof(testItem), testHash, testEquals, testDestroy1);
        size_t last_index, stop_index, prev_index, first_index;
        set_insert(set, &item1);
        stop_index = set_stop(set);
        last_index = set_last(set);
        first_index = set_first(set);
        prev_index = set_prev(set, last_index);
        assert(first_index == last_index);
        assert(first_index != stop_index);
        assert(set_last(set) == set_first(set));
        assert(stop_index == prev_index);
        assert(set_count(set) == 1);
        set_destroy(set, testDestroy1);
    }

    ///TEST 4///
    {
        testItem item = createItem1(100);
        void *set = set_create(sizeof(testItem), testHash, testEquals);
        set = set_init(set, sizeof(testItem), testHash, testEquals, testDestroy1);
        size_t stop_index, prev_index, first;
        set_insert(set, &item);
        stop_index = set_stop(set);
        first = set_first(set);
        prev_index = set_prev(set, first);
        assert(set_last(set) == set_first(set));
        assert(stop_index == prev_index);
        assert(testEquals(&item, set_current(set, first)));
        assert(set_count(set) == 1);
        set_destroy(set, testDestroy1);
    }

    _CrtDumpMemoryLeaks();
    return 0;
}
