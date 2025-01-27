#include <stdio.h>

double polynom(double x, double a[], int n) {
    double result = 0.0;

    for (int i = n; i >= 0; i--) {
        result = result * x + a[i];
    }

    return result;
}

// Пример использования
int main() {
    double coefficients[] = {1, -2, 3, 4}; // Многочлен 4x^3 + 3x^2 - 2x + 1
    int degree = 3; // Степень многочлена
    double x = 2.0; // Точка, в которой вычисляется значение

    double value = polynom(x, coefficients, degree);
    printf("Значение многочлена в точке x = %.2f: %.2f\n", x, value);

    return 0;
}
