#include <stdlib.h>
#include <string.h>
#include "darray.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

typedef struct {
    size_t capacity; // текущая ёмкость массива (количество элементов, которые могут быть хранены без перевыделения памяти).
    size_t itemSize; // размер одного элемента в байтах.
    size_t count; // количество элементов, фактически содержащихся в массиве.
    void *data; // указатель на область памяти, где хранятся элементы массива.
} DArray;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Создать новый пустой динамический массив. Размер элемента -- itemSize байт. */
void* darray_create(size_t itemSize) {
    if (itemSize == 0) {
        return NULL;
    }
    DArray* darray = malloc(sizeof(DArray));
    if (!darray)
        return NULL;
    darray->capacity = 1;
    darray->itemSize = itemSize;
    darray->count = 0;
    darray->data = malloc(darray->capacity * itemSize);
    if (!darray->data) {
        free(darray);
        return NULL;
    }
    return darray;
}

/* Удалить существующий динамический массив. Если указана функция destroy, то вызвать
 * её для каждого удаляемого элемента. */
void darray_destroy(void* darray, void(*destroy)(void*)) {
    if (!darray) {
        return;
    }
    darray_clear(darray, destroy);
    free(((DArray*)darray)->data);
    free(darray);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Инициализировать динамический массив. Размер элемента -- itemSize байт. Если darray
 * содержит элементы, то сначала удалить все элементы, потом инициализировать динамический
 * массив с учетом новых параметров. Если указана функция destroy, то вызвать её для
 * каждого удаляемого элемента. */
void* darray_init(void* darray, size_t itemSize, void(*destroy)(void*)) {
    if (!darray || itemSize == 0)
        return NULL;
    darray_clear(darray, destroy);
    ((DArray*)darray)->itemSize = itemSize;
    return darray;
}

/* Удалить все элементы из динамического массива. Если указана функция destroy, то вызвать
 * её для каждого удаляемого элемента. */
void darray_clear(void* darray, void(*destroy)(void*)) {
    if (!darray) {
        return;
    }
    if (destroy) {
        for (size_t i = 0; i < ((DArray *) darray)->count; i++) {
            destroy(((char *) ((DArray *) darray)->data) + i * ((DArray *) darray)->itemSize);
        }
    }
    ((DArray*)darray)->count = 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Количество элементов в массиве. В случае, если darray равен NULL, возвращает константу
 * INVALID. */
size_t darray_count(const void* darray) {
    return darray ? ((DArray*)darray)->count : ~((size_t)0);
}

/* Получить элемент по индексу в массиве. */
void* darray_item(void* darray, size_t i) {
    if (!darray || i == darray_stop(darray) || i >= ((DArray*)darray)->count) {
        return NULL;
    }
    return ((char*)((DArray*)darray)->data) + i * ((DArray*)darray)->itemSize;
}

/* Добавить элемента в конец динамического массива. В случае успеха, функция возвращает
 * указатель на добавленный элемент, иначе -- NULL. */
void* darray_add(void* darray) {
    if (!darray) {
        return NULL;
    }
    DArray* da = (DArray*)darray;
    if (da->count == da->capacity) {
        size_t newCapacity = da->capacity * 2;
        void* newData = realloc(da->data, newCapacity * da->itemSize);
        if (!newData) {
            return NULL;
        }
        da->data = newData;
        da->capacity = newCapacity;
    }
    return ((char*)da->data) + (da->count++) * da->itemSize;
}

/* Вставить элемент в середину динамического массива. Новый элемент должен иметь индекс i.
 * В случае успеха, функция возвращает указатель на добавленный элемент, иначе - NULL. */
void* darray_insert(void* darray, size_t i) {
    if (!darray || i == darray_stop(darray) || i > ((DArray*)darray)->count) {
        return NULL;
    }
    DArray* da = (DArray*)darray;
    if (da->count == da->capacity) {
        size_t newCapacity = da->capacity * 2;
        void* newData = realloc(da->data, newCapacity * da->itemSize);
        if (!newData) {
            return NULL;
        }
        da->data = newData;
        da->capacity = newCapacity;
    }
    memmove((char*)darray_item(da, i + 1), (char*)darray_item(da, i), (da->count - i) * da->itemSize);
    da->count++;
    return ((char*)da->data) + i * da->itemSize;
}

/* Удалить из массива элемент по индексу. Если указана функция destroy, то вызвать её для
 * удаляемого элемента. */
void darray_remove(void* darray, size_t i, void(*destroy)(void*)) {
    if (!darray || i == darray_stop(darray) || i >= ((DArray*)darray)->count) {
        return;
    }
    DArray* da = (DArray*)darray;
    if (destroy) {
        destroy(((char*)da->data) + i * da->itemSize);
    }
    memmove((char*)darray_item(da, i), (char*)darray_item(da, i + 1), (da->count - i - 1) * da->itemSize);
    da->count--;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Идентификатор для первого элемента из динамического массива. Идентификатор становится не
 * валидным при модификации динамического массива. */
size_t darray_first(const void* darray) {
    if (!darray || ((DArray *) darray)->count == 0) {
        return darray_stop(darray);
    }
    return 0;
}

/* Идентификатор для последнего элемента из динамического массива. Идентификатор становится
 * не валидным при модификации динамического массива. */
size_t darray_last(const void* darray) {
    if (!darray || ((DArray *) darray)->count == 0) {
        return darray_stop(darray);
    }
    return ((DArray *) darray)->count - 1;
}

/* По идентификатору текущего элемента получить идентификатор следующего элемента массива. */
size_t darray_next(const void* darray, size_t item_id) {
    if (!darray || item_id >= ((DArray*)darray)->count - 1) {
        return darray_stop(darray);
    }
    return item_id + 1;
}

/* По идентификатору текущего элемента получить идентификатор предыдущего элемента массива. */
size_t darray_prev(const void* darray, size_t item_id) {
    if (!darray || item_id <= 0) {
        darray_stop(darray);
    }
    return item_id - 1;
}

/* Идентификатор, получаемый при попытке получить доступ к элементу за пределами массива. */
size_t darray_stop(const void* darray) {
    (void) darray;
    return ~((size_t)0);
}

/* Получить указатель на элемент массива по его идентификатору. */
void* darray_current(const void* darray, size_t item_id) {
    return darray_item((void*)darray, item_id);
}

/* Удаление элемента по его идентификатору. Если указана функция destroy, то вызвать её для
 * удаляемого элемента. После удаления элемента из массива, идентификаторы любых элементов
 * из этого массива становятся не валидным. */
void darray_erase(void* darray, size_t item_id, void(*destroy)(void*)) {
    darray_remove(darray, item_id, destroy);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */