#include <stdio.h>

double power(double x, int n) {
    double result = 1.0;
    int exponent = (n < 0) ? -n : n; // Работаем с положительным показателем

    while (exponent > 0) {
        if (exponent % 2 == 1) { // Если текущий бит 1, умножаем на x
            result *= x;
        }
        x *= x;      // Умножаем x на себя (x^2, x^4, x^8 и т. д.)
        exponent /= 2; // Сдвигаем степень вправо
    }

    return (n < 0) ? 1.0 / result : result; // Если n отрицательное, возвращаем обратное число
}

int main() {
    double x;
    int n;

    printf("Введите число и степень: ");
    scanf("%lf %d", &x, &n);

    printf("%.6lf^%d = %.6lf\n", x, n, power(x, n));
    return 0;
}
