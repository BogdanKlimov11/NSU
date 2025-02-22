#include <malloc.h>
#include <assert.h>
#include <math.h>
#include <memory.h>

#include "darray.h"

#define OUT_OF_ARRAY ((size_t)-2)
typedef void* Data;

typedef struct D_array {
    Data a;           // pointer on allocated memory
    size_t size;      // for memory
    size_t itemSize;  // size of element
    size_t n;         // number of elements
} D_array;

void* darray_create(size_t itemSize) {
    if (!itemSize)
        return NULL;
    
    D_array* dArray = (D_array*)malloc(sizeof(D_array));
    if (!dArray)
        return NULL;
    
    dArray->n = 0;
    dArray->size = 10;
    dArray->itemSize = itemSize;
    dArray->a = malloc(dArray->itemSize * dArray->size);
    
    if (!dArray->a) {
        free(dArray);
        return NULL;
    }
    return dArray;
}

void darray_destroy(void* darray, void (*destroy)(void*)) {
    if (darray) {
        D_array* dArray = (D_array*)darray;
        if (destroy && dArray->n != 0) {
            for (size_t i = 0; i < dArray->n; i++) {
                void* ptr = (char*)(dArray->a) + i * dArray->itemSize;
                destroy(ptr);
            }
        }
        free(dArray->a);
        free(dArray);
    }
}

void resize(void* darray, size_t size_new) {
    if (darray && size_new > 0) {
        D_array* dArray = (D_array*)darray;
        Data new_arr = realloc(dArray->a, size_new * sizeof(dArray->itemSize));
        if (new_arr) {
            dArray->size = size_new;
            dArray->a = new_arr;
        }
    }
}

void* darray_init(void* darray, size_t itemSize, void (*destroy)(void*)) {
    if (!darray)
        return NULL;
    
    darray_clear(darray, destroy);
    D_array* dArray = (D_array*)darray;
    free(dArray->a);
    dArray->n = 0;
    dArray->size = 10;
    dArray->itemSize = itemSize;
    dArray->a = malloc(dArray->itemSize * dArray->size);
    
    if (!dArray->a) {
        free(dArray);
        return NULL;
    }
    return dArray;
}

void darray_clear(void* darray, void (*destroy)(void*)) {
    if (darray) {
        D_array* dArray = (D_array*)darray;
        if (destroy) {
            for (size_t i = 0; i < dArray->n; i++) {
                void* ptr = (char*)(dArray->a) + (dArray->itemSize * i);
                destroy(ptr);
            }
        }
        dArray->n = 0;
    }
}

size_t darray_count(struct D_array* darray) {
    if (!darray)
        return INVALID;
    return darray->n;
}

void* darray_item(void* darray, size_t i) {
    if (!darray)
        return NULL;
    
    D_array* dArray = (D_array*)darray;
    if (i < dArray->n && dArray->a)
        return (char*)(dArray->a) + (dArray->itemSize * i);
    
    return NULL;
}

void* darray_add(void* darray) {
    if (!darray)
        return NULL;
    
    D_array* dArray = (D_array*)darray;
    if (dArray->n == dArray->size)
        resize(darray, dArray->size * 2);
    
    void* pt = (char*)dArray->a + (dArray->n) * dArray->itemSize;
    dArray->n++;
    return pt;
}

void* darray_insert(void* darray, size_t i) {
    if (!darray)
        return NULL;
    
    D_array* dArray = (D_array*)darray;
    if (i > dArray->n || !dArray->a)
        return NULL;
    
    if (dArray->n == i)
        return darray_add(dArray);
    
    if (dArray->n == dArray->size)
        resize(darray, dArray->size * 2);
    
    void* ptr_temp = (char*)dArray->a + (i * dArray->itemSize);
    size_t section = (dArray->n - i) * dArray->itemSize;
    memmove((char*)ptr_temp + dArray->itemSize, ptr_temp, section);
    dArray->n++;
    return ptr_temp;
}

void darray_remove(void* darray, size_t i, void (*destroy)(void*)) {
    if (darray) {
        D_array* dArray = (D_array*)darray;
        if (i < dArray->n) {
            void* ptr_temp = (char*)dArray->a + (i * dArray->itemSize);
            if (destroy)
                destroy(ptr_temp);
            size_t section = (dArray->n - 1 - i) * dArray->itemSize;
            memcpy(ptr_temp, (char*)ptr_temp + dArray->itemSize, section);
            dArray->n--;
        }
    }
}

size_t darray_first(const void* darray) {
    return (darray && ((D_array*)darray)->n > 0) ? 0 : OUT_OF_ARRAY;
}

size_t darray_last(const void* darray) {
    return (darray && ((D_array*)darray)->n > 0) ? ((D_array*)darray)->n - 1 : OUT_OF_ARRAY;
}

size_t darray_next(const void* darray, size_t item_id) {
    D_array* dArray = (D_array*)darray;
    return (darray && item_id + 1 < dArray->n) ? item_id + 1 : OUT_OF_ARRAY;
}

size_t darray_prev(const void* darray, size_t item_id) {
    return (darray && item_id >= 1) ? item_id - 1 : OUT_OF_ARRAY;
}

size_t darray_stop(const void* darray) {
    return OUT_OF_ARRAY;
}

void* darray_current(const void* darray, size_t item_id) {
    D_array* dArray = (D_array*)darray;
    return (darray && item_id < dArray->n) ? (char*)dArray->a + dArray->itemSize * item_id : NULL;
}

void darray_erase(void* darray, size_t item_id, void (*destroy)(void*)) {
    darray_remove(darray, item_id, destroy);
}
