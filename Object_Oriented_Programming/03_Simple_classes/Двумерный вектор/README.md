# Задача «Двумерный вектор»

Реализовать класс `Vector` для работы с простыми двумерными векторами на
основе `double`. Класс представляет удобный интерфейс для выполнения всех
базовых операций над векторами на плоскости: сложение, вычитание, умножение
на скаляр, скалярное произведение, вычисление проекций и углов между
векторами, аффинные преобразования, и т.п.

### Постановка задачи

Реализовать класс `Vector` в соответствии с приведённым ниже интерфейсом.

```cpp
class Vector
{
public:
  Vector(void);
  explicit Vector(double x, double y);

public:
  Vector(const Vector & that);
  Vector & operator =(const Vector & that);

public:
  Vector makePolar(double rad, double alpha);

public:
  double x(void);
  double y(void);
  void x(double newX);
  void y(double newY);

public:
  Vector operator + (const Vector & that);
  Vector operator - (const Vector & that);
  double operator * (const Vector & that);
  Vector operator * (const double & that);
  Vector operator / (const double & that);

public:
  Vector & operator += (const Vector & that);
  Vector & operator -= (const Vector & that);
  Vector & operator *= (const double & that);
  Vector & operator /= (const double & that);

public:
  Vector operator -();

public:
  bool operator == (const Vector & that);
  bool operator != (const Vector & that);

public:
  Vector & rotate(double angle);
  Vector & rotate(int quad);
  double module2(void);
  double angle(void);
  double angle(const Vector & that);
  double length(void);
  double projection(const Vector & base);
  Vector & normalize(void);
  Vector & transformTo(const Vector & e1, const Vector & e2);
  Vector & transformFrom(const Vector & e1, const Vector & e2);

private:
  // поля...
};

Vector operator * (const double & lhs, const Vector & rhs);

Vector rotate(const Vector & v, double angle);
Vector rotate(const Vector & v, int quad);
double module2(const Vector & v);
double length(const Vector & v);
double angle(const Vector & v);
double angle(const Vector & v1, const Vector & v2);
double projection(const Vector & v, const Vector & base);
Vector normalize(const Vector & v);
Vector transformTo(const Vector & v, const Vector & e1, const Vector & e2);
Vector transformFrom(const Vector & v, const Vector & e1, const Vector & e2);
```

### Примечания

1. Для `public` и `protected` методов класса не допускается изменение
   названий, а также типов и количества аргументов. Однако, в приведённых
   фрагментах кода намеренно опущен ряд спецификаторов `const` и `static`.
   При реализации необходимо вернуть их на место в соответствии с логикой
   работы класса.

2. Ряд методов класса и функций на первый взгляд имеет одинаковый
   функционал. Однако это не так. Одни операции изменяет объект к которому
   применены, другие оставляют объект неизменным и создают изменённую копию.
   Приведённый интерфейс позволяет однозначно отделить первые от вторых.

3. Поворот вектора на произвольный угол — это всегда операция с потерей
   точности. Базовые математические функции принимают угловые аргументы в
   радианах, а значит, в частности, поворот на $90$ градусов всегда будет
   происходить с погрешностью. В то же время, для поворотов на угол кратный
   $90$ градусам никакая тригонометрия на самом деле не нужна. Поэтому в
   приведённом интерфейсе есть $2$ варианта функции поворота.

4. Переход из одного базиса в другой представлен $2$ функциями / методами:
   `transformTo(e1, e2)` и `transformFrom(e1, e2)`. Разберём их подробнее.
   Пусть в базисе `(i1, i2)` записан вектор `v`. Пусть в другом базисе
   `(e1, e2)` этот же вектор имеет запись `u`. Пусть вектора `e1` и `e2` в
   базисе `(i1, i2)` записываются как `e1_i` и `e2_i` соответственно. И
   наоборот, вектора `i1` и `i2` в базисе `(e1, e2)` записываются как `i1_e`
   и `i2_e`. Тогда переход из одного базиса в другой будет осуществляться
   следующим вызовом: `v.transformTo(e1_i, e2_i)` → `u`. Обратный переход
   может быть осуществлён двумя способами, либо через вызов
   `u.transformTo(i1_e, i2_e)` → `v`, либо через вызов
   `u.transformFrom(e1_i, e2_i)` → `v`, в зависимости от того, какое
   представление базиса более удобно.

### Описание интерфейса

* `Vector(void)` — Конструктор по-умолчанию. Инициализация нового экземпляра
  класса `Vector` нулевыми значениями.

* `Vector(double x, double y)` — Покомпонентная инициализация нового
  экземпляра класса `Vector`.

* `Vector(const Vector & that)` — Конструктор копии.

* `Vector & operator =(const Vector & that)` — Оператор присваивания.

* `Vector makePolar(double rad, double alpha)` — Инициализация нового экземпляра
  класса `Vector` из радиуса `rad` и угла `alpha` к оси `X`.

* `double x(void)` — доступ к `X`-компоненту вектора.

* `double y(void)` — доступ к `Y`-компоненту вектора.

* `void x(double newX)` — Модификация `X`-компонента вектора.

* `void y(double newY)` — Модификация `Y`-компонента вектора.

* `Vector operator + (const Vector & that)` — Операция сложения векторов.

* `Vector operator - (const Vector & that)` — Операция вычитания векторов.

* `double operator * (const Vector & that)` — Операция скалярного произведения
  векторов.

* `Vector operator * (const double & that)` — Операция умножения вектора на
  скаляр.

* `Vector operator / (const double & that)` — Операция деления вектора на скаляр.

* `Vector & operator += (const Vector & that)` — Операция добавления вектора к
  данному.

* `Vector & operator -= (const Vector & that)` — Операция вычитания вектора из
  данного.

* `Vector & operator *= (const double & that)` — Операция умножения данного вектора
  на скаляр.

* `Vector & operator /= (const double & that)` — Операция деления данного вектора на
  скаляр.

* `Vector operator -()` — Операция отрицания вектора.

* `bool operator == (const Vector & that)` — Операция проверки векторов на равенство.

* `bool operator != (const Vector & that)` — Операция проверки векторов на неравенство.

* `Vector & rotate(double angle)` — Метод для поворота вектора на произвольный угол
  `angle` (в радианах).

* `Vector & rotate(int quad)` — Метод для поворота вектора на угол, кратный $90$
  градусам (`+` — поворот против часовой, `-` — по часовой).

* `double module2(void)` — Метод, вычисляющий квадрат модуля вектора.

* `double length(void)` — Метод, вычисляющий длину вектора.

* `double angle(void)` — Метод, вычисляющий угол между данным вектором и осью `X`.

* `double angle(const Vector & that)` — Метод, вычисляющий угол между данным вектором
  и вектором `that`.

* `double projection(const Vector & base)` — Метод, вычисляющий проекцию данного
  вектора на заданный `base`.

* `Vector & normalize(void)` — Нормализация вектора (масштабирование к единичной
  длине).

* `Vector & transformTo(const Vector & e1, const Vector & e2)` — Преобразование
  текущего вектора в другую систему координат с базисом `e1` и `e2`.

* `Vector & transformFrom(const Vector & e1, const Vector & e2)` — Преобразование
  текущего вектора из системы координат с базисом `e1` и `e2`.

### Стандартные операторы, принимающие в качестве аргументов Vector

* `Vector operator * (const double & lhs, const Vector & rhs)` — Оператор умножения
  скаляра на вектор.

### Функции, принимающие в качестве аргументов `Vector`

* `Vector rotate(const Vector & v, double angle)` — Функция поворота вектора `v`
  на угол `angle`, заданный в радианах.

* `Vector rotate(const Vector & v, int quad)` — Функция поворота вектора `v` на
  угол кратный $90$ градусам.

* `double module2(const Vector & v)` — Функция вычисления квадрата модуля вектора
  `v`.

* `double length(const Vector & v)` — Функция, вычисляющая длину вектора `v`.

* `double angle(const Vector & v)` — Функция вычисления угла между вектором `v` и
  осью `X`.

* `double angle(const Vector & v1, const Vector & v2)` — Функция вычисления угла
  между векторами `v1` и `v2`.

* `double projection(const Vector & v, const Vector & base)` — Функция вычисления
  проекции вектора `v` на вектор `base`.

* `Vector normalize(const Vector & v)` — Функция нормализации векторами `v`.

* `Vector transformTo(const Vector & v, const Vector & e1, const Vector & e2)` —
  Функция преобразования вектора `v` в систему координат с базисом `e1` и `e2`.

* `Vector transformFrom(const Vector & v, const Vector & e1, const Vector & e2)` —
  Функция преобразования вектора `v` из системы координат с базисом `e1` и `e2`.

### Требования

* К `public` методам, класса `Vector` необходимо добавить в нужных местах
  спецификаторы `static` и `const`.

* Нельзя менять названия `public` и `protected` методов класса, а также типы
  и количество аргументов в них.

* Названия header и source файлов следующие: `vector.hpp`, `vector.cpp`.

* Написать unit test'ы для реализованного класса.
