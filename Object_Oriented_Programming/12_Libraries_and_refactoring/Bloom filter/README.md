# Задача «Bloom filter»

### Справка

<a href="https://ru.wikipedia.org/wiki/Фильтр_Блума">Описание</a>

### Задание

Произвести рефакторинг уже существующей реализации модуля bloom filter'a.

### Исходный код:

<a href="http://sourceforge.net/p/c-algorithms/code/ci/master/tree/src/bloom-filter.h">bloom-filter.h</a>

<a href="http://sourceforge.net/p/c-algorithms/code/ci/master/tree/src/bloom-filter.c">bloom-filter.c</a>

### Необходимо:

1. Сделать из C модуля С++ класс

2. Избавиться от аллокаций памяти в стиле C

3. Избавиться от define'ов (объявление констант) и глобальных
   переменных

4. Разнести на `.hpp` и `.сpp` файлы

5. Уйти от работы с указателями в пользу ссылок, где это возможно

6. Заменить с-style массивы

### Примечание
Чтобы собрать этот код, нужно в настройках файла выставить Compile as C Code. (Щелчок по файлу -> C/C++ -> Advanced -> Compile As)
