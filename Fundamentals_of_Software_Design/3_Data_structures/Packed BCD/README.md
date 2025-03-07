# Задача «Packed BCD»

### Условие

Реализовать модуль работы с числами в двоично-десятичном коде (формат
Packed BCD).

Число хранится в 32-битном беззнаковом целом, имея следующий формат DD
DD DD DS, где D и S означают группы по 4 бит (тетрады). Семь тетрад
DDDDDDD хранят семь десятичных цифр числа, последняя тетрада S хранит
его знак.

Значение D находится в диапазоне 0..9, значение S — либо `0xC (12)`
(знак +), либо `0xD (13)` (знак -).

### Функции и структуры данных

```python
class Bcd:
    INVALID_BCD = 0xFFFFFFFF

    @staticmethod
    def bcd_add(n1: int, n2: int) -> int:
        pass

    @staticmethod
    def bcd_sub(n1: int, n2: int) -> int:
        pass

    @staticmethod
    def bcd_mul(n1: int, n2: int) -> int:
        pass

    @staticmethod
    def bcd_div(n1: int, n2: int) -> tuple[int, int]:
        pass

    @staticmethod
    def bcd_compare(n1: int, n2: int) -> int:
        pass

    @staticmethod
    def bcd_from_int(n: int) -> int:
        pass

    @staticmethod
    def bcd_to_int(n: int) -> int:
        pass

    @staticmethod
    def bcd_to_str(n: int) -> str:
        pass
```

```c
typedef unsigned int Bcd;

#define INVALID_BCD 0xFFFFFFFFU

/* Арифметика */
Bcd bcd_add(Bcd n1, Bcd n2);
Bcd bcd_sub(Bcd n1, Bcd n2);
Bcd bcd_mul(Bcd n1, Bcd n2);
void bcd_div(Bcd n1, Bcd n2, Bcd *pq, Bcd *pr);
/* pq и pr -- указатели на частное и остаток */

/* Сравнение */
int bcd_compare(Bcd n1, Bcd n2);
/* возвращает -1, 0 или 1 (n1 < n2, n1 == n2, n1 > n2) */

/* Преобразования */
Bcd bcd_from_int(int n);
int bcd_to_int(Bcd n);
char *bcd_to_str(Bcd n, char *buf, int bufsize);
/* если число поместилось в buf (его размер -- bufsize), 
   возвращает buf, в противном случае записывает bufsize-1 символов,
   завершает нулевым символом и возвращает 0 */
```

### Замечания по интерфейсу

* Если на вход любой функции поступает некорректное значение, результатом
  будет специальная константа `INVALID_BCD` (кроме `bcd_to_str`, которая
  может сделать пустую строку).

* В `bcd_div` для сигнализации деления на 0 можно вернуть частное и остаток,
  равные 0 (т.е. `0xC`).

### Замечания по реализации

* Арифметику с числами выполнять непосредственно, без преобразований в `int`.
  Для этого воспользоваться двоично-десятичной коррекцией с добавлением
  значения 6 (см. лекцию № 3). Умножение проводить столбиком, деление —
  уголком.
