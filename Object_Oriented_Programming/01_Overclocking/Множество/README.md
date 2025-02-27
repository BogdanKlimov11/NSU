# Задача «Множество»

Реализовать на языке C модуль для работы с множеством.

Строго заданный интерфейс:

```c
#include <stdbool.h> // bool
#include <stddef.h>  // size_t

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
void set_erase(void* set, size_t item_id, void (*destroy)(void*));
```

* `set_create` — Создать новое пустое множество. Размер элемента —
  `itemSize`, для обработки элементов использовать функцию хеширования
  `hash`, и функцию проверки на равенство `equals`.

* `set_destroy` — Удалить существующее множество. Если указана функция
  `destroy`, то вызвать её для каждого удаляемого элемента множества.

* `set_init` — Инициализировать множество новыми параметрами. Если
  `set` содержит элементы, то сначала удалить все элементы, потом
  инициализировать множество с учетом новых параметров. Размер элемента
  — `itemSize`, для обработки элементов использовать функцию хеширования
  `hash`, и функцию проверки на равенство `equals`. Если указана функция
  `destroy`, то вызвать её для каждого удаляемого элемента.

* `set_clear` — Удалить все элементы из множества. Если указана функция
  `destroy`, то вызвать её для каждого удаляемого элемента множества.

* `set_count` — Количество элементов во множестве. В случае, если `set`
  равен `NULL`, возвращает `INVALID` константу.

* `set_contains` — проверить наличие во множестве заданного элемента.

* `set_insert` — Добавить новый элемент. В случае успеха, функция возвращает
  `true`, если такой элемент уже существует — `false`.

* `set_remove` — Найти элемент и удалить из множества. Если указана функция
  `destroy`, то вызвать её для удаляемого элемента `setItem`.

* `set_first` — Идентификатор для первого элемента множества. Идентификатор
  может стать невалидным при модификации множества.

* `set_last` — Идентификатор для последнего элемента множества. Идентификатор
  может стать невалидным при модификации множества.

* `set_next` — По идентификатору текущего элемента получить идентификатор
  следующего элемента множества.

* `set_prev` — По идентификатору текущего элемента получить идентификатор
  предыдущего элемента множества.

* `set_stop` — Идентификатор, получаемый при попытке обратиться к элементу
  за пределами множества.

* `set_current` — Получить указатель на элемент по его идентификатору.

* `set_erase` — Удаление элемента множества по его идентификатору. Если указана
  функция `destroy`, то вызвать её для удаляемого элемента множества. После
  удаления элемента, идентификаторы любых элементов из этого множества могут
  стать невалидным.

```c
//Пример использования
#include <string.h>
#include <assert.h>

#include "set.h"

typedef struct {
    char name[10];
} KeyValue;

static size_t hash(const void* ptr) {
    const int p = 31;
    const int m = 1e9 + 9;
    size_t hash_value = 0;
    size_t p_pow = 1;

    const KeyValue* keyValue = (const KeyValue*)ptr;

    for (size_t i = 0; i < 10; ++i) {
        const char c = keyValue->name[i];
        hash_value = (hash_value + (c - 'a' + 1) * p_pow) % m;
        p_pow = (p_pow * p) % m;
    }

    return hash_value;
}

static bool equals(const void* lhsp, const void* rhsp) {
    const KeyValue* lhs = (const KeyValue*)lhsp;
    const KeyValue* rhs = (const KeyValue*)rhsp;

    return 0 == strcmp(lhs->name, rhs->name);
}

int main(int argc, char* argv[])
{
    //Создаем множество с элементами типа KeyValue;
    void* set = set_create(sizeof(KeyValue), hash, equals);

    assert(0 == set_count(set));
    assert(set_stop(set) == set_first(set));

    //Создаем ключ-значение для множества
    const KeyValue keyValue = { "Key-value" };

    //Добавляем ключ-значение
    const bool isCreated = (KeyValue*)set_insert(set, &keyValue);
    assert(true == isCreated);

    assert(true == set_contains(set, &keyValue));
    const KeyValue* item = (const KeyValue*)set_current(set, set_first(set));

    assert(0 == strcmp(item->name, keyValue.name));

    assert(set_last(set) == set_first(set));
    assert(set_next(set, set_first(set)) == set_stop(set));

    set_destroy(set, NULL);

    return 0;
}
```
