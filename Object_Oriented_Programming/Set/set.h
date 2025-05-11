#ifndef SET_SET_H
#define SET_SET_H

#include <stdbool.h> 
#include <stddef.h>  

static const size_t INVALID = ~((size_t)0);

void* set_create(size_t itemSize, size_t hash(const void*), bool (*equals)(const void*, const void*));
void set_destroy(void* set, void (*destroy)(void*));
void* set_init(void* set, size_t itemSize, size_t hash(const void*), bool (*equals)(const void*, const void*),
               void (*destroy)(void*));
void set_clear(void* set, void (*destroy)(void*));
size_t set_count(const void* set);
bool set_contains(const void* set, const void* item);
bool set_insert(void* set, const void* item);
void set_remove(void* set, const void* item, void (*destroy)(void*));
size_t set_first(const void* set);
size_t set_last(const void* set);
size_t set_next(const void* set, size_t item_id);
size_t set_prev(const void* set, size_t item_id);
size_t set_stop(const void* set);
const void* set_current(const void* set, size_t item_id);
void set_erase(void* set, size_t erase_id, void (*destroy)(void*));

#endif //SET_SET_H
