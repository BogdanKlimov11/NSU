#include <stdio.h>
#include <stdlib.h>

// Структура для хранения простых множителей
struct PrimeFactor {
    unsigned long prime;  // Простое число (множитель)
    unsigned long power;  // Степень этого множителя
    struct PrimeFactor *next;  // Указатель на следующий элемент списка
};

// Функция разложения числа на простые множители
struct PrimeFactor *prime_decompose(unsigned long n) {
    struct PrimeFactor *head = NULL, *tail = NULL;

    // Особые случаи: для 0 и 1 возвращаем число с мощностью 1
    if (n == 0 || n == 1) {
        struct PrimeFactor *node = (struct PrimeFactor *)malloc(sizeof(struct PrimeFactor));
        node->prime = n;
        node->power = 1;
        node->next = NULL;
        return node;
    }

    // Разложение числа на простые множители
    for (unsigned long i = 2; i * i <= n; ++i) {
        unsigned long power = 0;
        while (n % i == 0) {
            n /= i;
            power++;
        }
        if (power > 0) {
            struct PrimeFactor *node = (struct PrimeFactor *)malloc(sizeof(struct PrimeFactor));
            node->prime = i;
            node->power = power;
            node->next = NULL;
            if (tail) {
                tail->next = node;
                tail = node;
            } else {
                head = tail = node;
            }
        }
    }

    // Если n больше 1, то это простое число
    if (n > 1) {
        struct PrimeFactor *node = (struct PrimeFactor *)malloc(sizeof(struct PrimeFactor));
        node->prime = n;
        node->power = 1;
        node->next = NULL;
        if (tail) {
            tail->next = node;
        } else {
            head = node;
        }
    }

    return head;
}

// Функция освобождения памяти, выделенной для списка
void prime_release(struct PrimeFactor *head) {
    struct PrimeFactor *current = head;
    while (current) {
        struct PrimeFactor *temp = current;
        current = current->next;
        free(temp);
    }
}

// Функция для печати списка множителей
void print_prime_factors(struct PrimeFactor *head) {
    struct PrimeFactor *current = head;
    while (current) {
        printf("%lu^%lu ", current->prime, current->power);
        current = current->next;
    }
    printf("\n");
}

int main() {
    unsigned long n;
    printf("Введите число для разложения: ");
    scanf("%lu", &n);

    struct PrimeFactor *factors = prime_decompose(n);
    print_prime_factors(factors);
    
    prime_release(factors);
    return 0;
}
