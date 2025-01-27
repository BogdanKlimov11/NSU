#include <stdio.h>
#include <stdbool.h>

void sieve_of_eratosthenes(int N) {
    // Массив для отметки простых чисел
    bool is_prime[N + 1];
    
    // Изначально все числа считаются простыми
    for (int i = 0; i <= N; i++) {
        is_prime[i] = true;
    }
    
    // 0 и 1 не являются простыми числами
    is_prime[0] = is_prime[1] = false;
    
    // Алгоритм решета Эратосфена
    for (int i = 2; i * i <= N; i++) {
        if (is_prime[i]) {
            for (int j = i * i; j <= N; j += i) {
                is_prime[j] = false;
            }
        }
    }
    
    // Выводим простые числа
    for (int i = 2; i <= N; i++) {
        if (is_prime[i]) {
            printf("%d ", i);
        }
    }
    printf("\n");
}

int main() {
    int N;
    printf("Введите число N: ");
    scanf("%d", &N);

    printf("Простые числа, меньшие или равные %d:\n", N);
    sieve_of_eratosthenes(N);

    return 0;
}
