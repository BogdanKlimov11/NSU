#ifndef DARRAY_H
#define DARRAY_H

#include <stdbool.h>	// bool
#include <stddef.h>		// size_t

static const size_t INVALID = ~((size_t)0);

void* darray_create(size_t itemSize);
void darray_destroy(void* darray, void(*destroy)(void*));

void* darray_init(void* darray, size_t itemSize, void(*destroy)(void*));
void darray_clear(void* darray, void(*destroy)(void*));

size_t darray_count(const void* darray);
void* darray_item(void* darray, size_t i);
void* darray_add(void* darray);
void* darray_insert(void* darray, size_t i);
void darray_remove(void* darray, size_t i, void(*destroy)(void*));

size_t darray_first(const void* darray);
size_t darray_last(const void* darray);
size_t darray_next(const void* darray, size_t item_id);
size_t darray_prev(const void* darray, size_t item_id);
size_t darray_stop(const void* darray);
void* darray_current(const void* darray, size_t item_id);
void darray_erase(void* darray, size_t item_id, void(*destroy)(void*));

#endif /* DARRAY_H: */