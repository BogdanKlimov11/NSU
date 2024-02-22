#include <assert.h>
#include <math.h>

#include "darray.h"

typedef struct {
    int array[8];
    float d_variable;
} Value;

int main()
{
    {
        //Создаем динамический массив с элементами типа Value;
        void *darray = darray_create(sizeof(Value));

        assert(0 == darray_count(darray));
        assert(darray_last(darray) == darray_first(darray));

        //Создаем объект для динамического массива
        Value value = {{1, 2, 3, 4, 5, 6, 7, 8}, 0.f};

        assert(darray_last(darray) == INVALID);

        //Добавляем новый элемент в динамический массив
        Value *insertedValue = (Value *) darray_add(darray);

        //Инициализируем добавленный элемент
        *insertedValue = value;

        Value *item = (Value *) darray_item(darray, 0);

        for (size_t i = 0; 8 > i; ++i) {
            assert(item->array[i] == value.array[i]);
        }

        assert(fabsf(item->d_variable - value.d_variable) < 1e-10f);
        assert(NULL == darray_item(darray, 1));

        assert(darray_next(darray, darray_first(darray)) == darray_stop(darray));

        darray_destroy(darray, NULL);
    }

    {
        // itemSize = 0
        void *darray = darray_create(0);
        assert(darray == NULL);
        darray_destroy(darray, NULL);
    }

    {
        // itemSize < 0
        void* darray = darray_create(-5);
        assert(darray == NULL);
        darray_destroy(darray, NULL);
    }

    {
        void *darray_1 = darray_create(sizeof(float));
        void *darray = darray_init(darray_1, sizeof(int), NULL);
        darray_destroy(darray_1, NULL);
        assert(darray != NULL);
    }

    {
        void *darray = darray_create(sizeof(int));

        assert(darray_first(darray) == darray_last(darray));

        assert(darray_insert(darray, -10) == NULL);
        assert(darray_insert(darray, 15) == NULL);

        *((int*)darray_insert(darray, 0)) = 5;
        assert(darray_first(darray) == darray_last(darray));

        *((int*)darray_insert(darray, 1)) = 9;
        assert(darray_first(darray) != darray_last(darray));

        *((int*)darray_insert(darray, 2)) = 13;
        *((int*)darray_insert(darray, 3)) = 16;
        assert(darray_count(darray) == 4);

        *((int*)darray_insert(darray, 4)) = 7;
        assert(darray_count(darray) == 5);

        *((int*)darray_insert(darray, 2)) = 1;
        assert(darray_count(darray) == 6);

        int item = (int)darray_last(darray);
        assert(item == 5);

        assert(darray_prev(darray, item) == 4);
        assert(darray_prev(darray, 0) == INVALID);
        assert(darray_next(darray, item) == INVALID);

        assert(darray_current(darray, 0) == darray_item(darray, 0));
        darray_erase(darray, 0, NULL);
        assert(darray_count(darray) == 5);
        assert(*((int*)darray_item(darray, 0)) == 9);

        darray_destroy(darray, NULL);
    }

    return 0;
}