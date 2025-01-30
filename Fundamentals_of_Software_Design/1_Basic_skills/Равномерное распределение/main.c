#include <stdio.h>
#include <stdlib.h>
#include <time.h>

void test_random_distribution(int n) {
    int count[101] = {0};  // Массив для подсчета частоты выпадений чисел

    for (int i = 0; i < n; i++) {
        int num = rand() % 101;  // Генерация случайного числа от 0 до 100
        count[num]++;
    }

    // Вывод статистики
    for (int i = 0; i <= 100; i++) {
        printf("%d: %d раз\n", i, count[i]);
    }
}

int main() {
    // Инициализация генератора случайных чисел
    srand(time(NULL));

    int n;
    printf("Введите количество случайных чисел для генерации: ");
    scanf("%d", &n);

    test_random_distribution(n);

    return 0;
}
