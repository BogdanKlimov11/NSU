# Задача «Двоичный поиск»

Реализовать поиск делением пополам в отсортированном массиве.
Функция принимает ключ `key` и находит индекс элемента, равного
ключу.

```python
def binary_search(arr, key):
    ...
```

```c
int binary_search(int arr[], int size, int key)
{
    /* .... */
}
```

Поведение функции проверить с помощью `assert` (особенно краевые
случаи: `size=0`, `size=1`, `size` четное, `size` нечетное,
элемента нет в массиве).
