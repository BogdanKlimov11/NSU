# Задача «2D-интерполяция»

Необходимо разработать программу двумерной интерполяции.

### Входные данные

Текстовый файл:

```bash
1.0 2.1 3.421
2.54 2.1 5.491
12.0 2.24 3.411
```

### Постановка задачи

Реализовать программу двумерной интерполяции, которая читает входной файл,
формирует `std::vector<double>`, который содержит в себе двумерный массив
отсчётов на равномерной сетке, размера `Nsrc * Nsrc`. Размеры определяются
из входного файла.

В результате работы должен сформироваться массив интерполированных значений,
соответствующий более мелкой сетке `Ndst * Ndst`. (`Ndst > Nsrc`)

Массив необходимо записать в выходной файл.

Передачу имен файлов и размера результирующего массива, реализовать через
аргументы командной строки.

### Формулы интерполяции

Желательно реализовать обе формулы.

* <a href="http://www.machinelearning.ru/wiki/index.php?title=Интерполяция_функций_двух_переменных,_проблема_выбора_узлов">Билинейная интерполяция</a>

* <a href="https://habr.com/ru/articles/111402/">Бикубическая интерполяция</a>

Выбор формулы, осуществляется, так-же, через командную строку.

### Выходные данные

Текстовый файл (не связан с вышеописанными входными данными)

```bash
1.0 2.1 3.421 1.3
2.54 2.1 5.491 1.4
12.0 2.24 3.411 1.5
12.0 2.24 3.411 1.5
```

### Тестирование

Для всех разработанных модулей должны быть созданы наборы unit тестов. Функции
ввода/вывода нужно тестировать с помощью `std::stringstream`.
