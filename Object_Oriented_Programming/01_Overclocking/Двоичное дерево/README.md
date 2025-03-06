# Задача «Двоичное дерево»

Реализовать на языке C модуль для работы с двоичным несбалансированным
деревом элементов.

Строго заданный интерфейс:

```c
//Файл 'btree.h'

#include <stdbool.h> // bool
#include <stddef.h>  // size_t

static const size_t INVALID = ~((size_t)0);

typedef
struct BTreeItem
{
    const void * key;
    void * value;
}
BTreeItem;


void * btree_create(size_t keySize, size_t valueSize, int(*compare)( const void *, const void * ));
void btree_destroy(void * btree, void(*destroy)( void * ));

void * btree_init(
    void * btree, 
    size_t keySize, 
    size_t valueSize, 
    int(*compare)( const void *, const void * ), 
    void(*destroy)( void * ));
void btree_clear(void * btree, void(*destroy)( void * ));

size_t btree_count(const void * btree);
void * btree_item(const void * btree, const void * key);
void * btree_insert(void * btree, const void * key, bool * createFlag);
void btree_remove(void * btree, const void * key, void(*destroy)( void * ));

size_t btree_first(const void * btree);
size_t btree_last(const void * btree);
size_t btree_next(const void * btree, size_t item_id);
size_t btree_prev(const void * btree, size_t item_id);
size_t btree_stop(const void * btree);
void * btree_current(const void * btree, size_t item_id);
void btree_erase(void * btree, size_t item_id, void(*destroy)( void * ));
```

* `INVALID` — код ошибки для не валидных ситуаций.

* `struct BTreeItem` — элемент двоичного дерева, содержащий в себе
  указатели на ключ и значение.

* `btree_create` — Создать новое пустое двоичное дерево. Размер ключа —
  `keySize`, размер значения элемента — `valueSize`, для упорядочивания
  элементов по ключу использовать функцию сравнения compare.

* `btree_destroy` — Удалить существующее двоичное дерево. Если указанафункция
  `destroy`, то вызвать её для каждого удаляемого элемента `btreeItem`.

* `btree_init` — Инициализировать двоичное дерево новыми параметрами. Размер
  ключа -- `keySize`, размер значения элемента — `valueSize`, для
  упорядочивания элементов по ключу использовать функцию сравнения `compare`.
  Если `btree` содержит элементы, то сначала удалить все элементы, потом
  инициализировать двоичное дерево с учетом новых параметров. Если указана
  функция `destroy`, то вызвать её для каждого удаляемого элемента
  `btreeItem`.

* `btree_clear` — Удалить все элементы из двоичного дерева. Если указана
  функция `destroy`, то вызвать её для каждого удаляемого элемента
  `btreeItem`.

* `btree_count` — Количество элементов в дереве. В случае, если `btree`
  равен `NULL`, возвращает константу `INVALID`.

* `btree_item` — Получить значение по заданному ключу. В случае наличия
  искомого ключа в дереве, функция возвращает указатель на значение связанное
  с этим ключом, иначе — `NULL`.

* `btree_insert` — Добавить значение по заданному ключу. В случае успеха,
  функция возвращает указатель на связанное с искомым ключом значение,
  иначе — `NULL`. Указатель `createFlag` не может быть равен `NULL`. При
  создании нового элемента дерева этот флаг будет установлен в `true`, при
  доступе к значению уже существовавшего элемента -- в `false`.

* `btree_remove` — Найти по ключу и удалить элемент из двоичного дерева.
  Если указана функция `destroy`, то вызвать её для удаляемого элемента
  `btreeItem`.

* `btree_first` — Идентификатор для первого элемента двоичного дерева.
  Идентификатор может стать не валидным при модификации дерева.

* `btree_last` — Идентификатор для последнего элемента двоичного дерева.
  Идентификатор может стать не валидным при модификации дерева.

* `btree_next` — По идентификатору текущего элемента получить идентификатор
  следующего элемента дерева.

* `btree_prev` — По идентификатору текущего элемента получить идентификатор
  предыдущего элемента дерева.

* `btree_stop` — Идентификатор, получаемый при попытке обратиться к элементу
  за пределами дерева.

* `btree_current` — Получить элемент `BTreeItem` по его идентификатору.

* `btree_erase` — Удаление элемента двоичного дерева по его идентификатору.
  Если указана функция `destroy`, то вызвать её для удаляемого элемента
  `btreeItem`. После удаления элемента, идентификаторы любых элементов из
  этого дерева могут стать невалидным.

```c
//Пример использования
#include <string.h>
#include <assert.h>
#include <math.h>

#include "btree.h"

typedef struct {
    int array[8];
    float d_variable;
} Value;

typedef struct {
    char name[10];
} Key;

static int compare(const void* lhsp, const void* rhsp) {
    const Key* lhs = (const Key*)lhsp;
    const Key* rhs = (const Key*)rhsp;

    return strcmp(lhs->name, rhs->name);
}

int main(int argc, char* argv[])
{
    //Создаем бинарное дерево: ключ - тип Key; значение - тип Value;
    void* btree = btree_create(sizeof(Key), sizeof(Value), compare);

    assert(0 == btree_count(btree));
    assert(btree_stop(btree) == btree_first(btree));

    //Создаем ключ и значение 
    Key key = { "key1" };
    Value value = { {1, 2, 3, 4, 5, 6, 7, 8}, 0.f };

    bool isCreated = false;

    //Добавляем по ключу значение
    Value* insertedValue = (Value*)btree_insert(btree, &key, &isCreated);
    assert(true == isCreated);
    *insertedValue = value;

    //Проверяем, что по добавленному ключу получим соответствующее значение
    Value* item = (Value*)btree_item(btree, &key);

    for (size_t i = 0; 8 > i; ++i) {
        assert(item->array[i] == value.array[i]);
    }

    assert(fabsf(item->d_variable - value.d_variable) < 1e-10f);

    assert(btree_last(btree) == btree_first(btree));
    assert(btree_next(btree, btree_first(btree)) == btree_stop(btree));

    btree_destroy(btree, NULL);

    return 0;
}
```
