# Задача «Частотный словарь»

Частотный словарь (или частотный список) — набор слов данного
языка (или подъязыка) вместе с информацией о частоте их
встречаемости.

Необходимо разработать программу, составляющую частотный словарь
на основе входного текста.

### Входные данные

Текстовый файл, содержащий большое количество слов.

### Задача

Разработать программу, которая подсчитывает, сколько раз каждое из
слов встречается во входном файле, и выводит результаты в другой
файл. Алгоритм должен иметь возможность использования различных
STL-контейнеров для своей работы (`std::list`, `std::vector`,
`std::map`).

### Пример вывода

Вывод в текстовый файл:

1. Время работы программы.
2. Вывод всех слов и количества их вхождений в формате: `“слово” -
   “количество вхождений”`.

```
Time - 100.2 sec
-----------------

my - 10
hey - 100
student - 1
...
```

### Тестирование

Для всех разработанных модулей должны быть созданы наборы unit тестов.
Функций ввода/вывода нужно тестировать с помощью `std::stringstream`.
