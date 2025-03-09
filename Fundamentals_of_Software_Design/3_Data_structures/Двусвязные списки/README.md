# Задача «Двусвязные списки»

Реализовать модуль работы с двусвязными списками.

Взять структуру и набор функций из задачи «Связанные списки», изменив
интерфейс с учетом большей симметричности двусвязного списка (пара
функций изменят семантику, пару функций можно добавить — подумайте,
какие именно).

В отличии от односвязного списка, потребуется дополнительная структура,
содержащая указатели на голову и хвост списка. Именно эта структура
будет видна пользователю,

Тесты на все функции с помощью `assert`.

Пример использования (вариант Python):

```python
import dlist

x = dlist.create()
dlist.prepend(x, "bb")
dlist.prepend(x, "aa")
dlist.append(x, "yy")
dlist.append(x, "zz")
dlist.get(x, 0); # => aa
dlist.get(x, 1); # => bb
dlist.get(x, 2); # => yy
dlist.get(x, 3); # => zz
```

Пример использования (вариант C):

```c
DList dl;
dlist_create(&dl);
dlist_prepend(&dl, "bb");
dlist_prepend(&dl, "aa");
dlist_append(&dl, "yy");
dlist_append(&dl, "zz");
dlist_nth(&dl, 0); // => aa
dlist_nth(&dl, 1); // => bb
dlist_nth(&dl, 2); // => yy
dlist_nth(&dl, 3); // => zz
dlist_destroy(&dl);
```
