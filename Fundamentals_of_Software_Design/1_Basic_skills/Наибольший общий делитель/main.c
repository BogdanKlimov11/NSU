#include <stdio.h>

// Функция для вычисления НОД по модифицированному алгоритму Евклида
int greatest_common_divisor(int a, int b) {
    // Убедимся, что числа положительные
    if (a < 0) a = -a;
    if (b < 0) b = -b;

    while (a != 0 && b != 0) {
        if (a >= b) {
            a %= b; // Остаток от деления a на b
        } else {
            b %= a; // Остаток от деления b на a
        }
    }
    return a == 0 ? b : a; // Если одно из чисел стало 0, возвращаем другое
}

int main() {
    int a, b;

    printf("Введите два целых числа: ");
    scanf("%d %d", &a, &b);

    int gcd = greatest_common_divisor(a, b);

    printf("НОД(%d, %d) = %d\n", a, b, gcd);

    return 0;
}
