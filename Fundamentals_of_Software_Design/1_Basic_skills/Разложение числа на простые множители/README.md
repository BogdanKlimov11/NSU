# Задача «Разложение числа на простые множители»

Реализовать функцию разложения числа на простые множители. Множители должны попадать в связный список.

```python
class PrimeFactor:
    def __init__(self, prime, power):
        self.prime = prime # множитель
        self.power = power # степень
        self.next = None

def prime_decompose(n):
    ...

def prime_release(head):
    ...
```

```c
struct PrimeFactor {
    unsigned long prime; /* множитель */
    unsigned long power; /* степень */
    struct PrimeFactor *next;
};

struct PrimeFactor *prime_decompose(unsigned long n);
void prime_release(struct PrimeFactor *head);
```

Функция `prime_release` освобождает память, выделенную под список функцией `prime_decompose`.

Для чисел $0$, $1$, а также всех простых чисел возвращаемый список будет состоять из одного-единственного элемента с самим числом и степенью $1$.
