# Задача «Динамический массив»

Реализовать на языке C модуль для работы динамическим массивом записей.

Строго заданный интерфейс:

```c
//darray.h

#include <stdbool.h> // bool
#include <stddef.h>  // size_t

static const size_t INVALID = ~((size_t)0);

void * darray_create(size_t itemSize);
void darray_destroy(void * darray, void(*destroy)( void * ));

void * darray_init(void * darray, size_t itemSize, void(*destroy)( void * ));
void darray_clear(void * darray, void(*destroy)( void * ));

size_t darray_count(const void * darray);
void * darray_item(void * darray, size_t i);
void * darray_add(void * darray);
void * darray_insert(void * darray, size_t i);
void darray_remove(void * darray, size_t i, void(*destroy)( void * ));

size_t darray_first(const void * darray);
size_t darray_last(const void * darray);
size_t darray_next(const void * darray, size_t item_id);
size_t darray_prev(const void * darray, size_t item_id);
size_t darray_stop(const void * darray);
void * darray_current(const void * darray, size_t item_id);
void darray_erase(void * darray, size_t item_id, void(*destroy)( void * ));
```

* `INVALID` — код ошибки для не валидных ситуаций.

* `darray_create` — Создать новый пустой динамический массив. Размер элемента
  — `itemSize` байт.

* `darray_destroy` — Удалить существующий динамический массив. Если указана функция
  `destroy`, то вызвать её для каждого удаляемого элемента.

* `darray_init` — Инициализировать динамический массив. Размер элемента — `itemSize`
  байт. Если `darray` содержит элементы, то сначала удалить все элементы, потом
  инициализировать динамический массив с учетом новых параметров. Если указана
  функция `destroy`, то вызвать её для каждого удаляемого элемента.

* `darray_clear` — Удалить все элементы из динамического массива. Если указана
  функция `destroy`, то вызвать её для каждого удаляемого элемента.

* `darray_сount` — Количество элементов в массиве. В случае, если darray равен `NULL`,
  возвращает константу `INVALID`.

* `darray_item` — Получить элемент по индексу в массиве.

* `darray_add` — Добавить элемента в конец динамического массива. В случае успеха,
  функция возвращает указатель на добавленный элемент, иначе — `NULL`.

* `darray_insert` — Вставить элемент в середину динамического массива. Новый элемент
  должен иметь индекс `i`. В случае успеха, функция возвращает указатель на
  добавленный элемент, иначе — `NULL`.

* `darray_remove` — Удалить из массива элемент по индексу. Если указана функция
  `destroy`, то вызвать её для удаляемого элемента.

* `darray_first` — Идентификатор для первого элемента из динамического массива.
  Идентификатор становится не валидным при модификации динамического массива.

* `darray_last` — Идентификатор для последнего элемента из динамического массива.
  Идентификатор становится не валидным при модификации динамического массива.

* `darray_next` — По идентификатору текущего элемента получить идентификатор
  следующего элемента массива.

* `darray_prev` — По идентификатору текущего элемента получить идентификатор
  следующего элемента массива.

* `darray_stop` — Идентификатор, получаемый при попытке получить доступ к элементу
  за пределами массива.

* `darray_current` — Получить указатель на элемент массива по его идентификатору.

* `darray_erase` — Удаление элемента по его идентификатору. Если указана функция
  `destroy`, то вызвать её для удаляемого элемента. После удаления элемента из
  массива, идентификаторы любых элементов из этого массива становятся не валидным.

```c
//Пример использования
#include <string.h>
#include <assert.h>
#include <math.h>

#include "darray.h"

typedef struct {
    int array[8];
    float d_variable;
} Value;

int main(int argc, char* argv[])
{
    //Создаем динамический массив с элементами типа Value;
    void* darray = darray_create(sizeof(Value));

    assert(0 == darray_count(darray));
    assert(darray_stop(darray) == darray_first(darray));

    //Создаем объект для динамического массива
    Value value = { {1, 2, 3, 4, 5, 6, 7, 8}, 0.f };

    //Добавляем новый элемент в динамический массив
    Value* insertedValue = (Value*)darray_add(darray);

    //Инициализируем добавленный элемент
    *insertedValue = value;

    Value* item = (Value*)darray_item(darray, 0);

    for (size_t i = 0; 8 > i; ++i) {
        assert(item->array[i] == value.array[i]);
    }

    assert(fabsf(item->d_variable - value.d_variable) < 1e-10f);
    assert(NULL == darray_item(darray, 1));

    assert(darray_next(darray, darray_first(darray)) == darray_stop(darray));

    darray_destroy(darray, NULL);

    return 0;
}
```
