#include <stdio.h>
#include <math.h>

// Функция для решения квадратного уравнения
void solve_quadratic(double a, double b, double c) {
    if (a == 0) {
        // Уравнение становится линейным: bx + c = 0
        if (b == 0) {
            if (c == 0) {
                printf("Бесконечное количество решений.\n");
            } else {
                printf("Решений нет.\n");
            }
        } else {
            double x = -c / b;
            printf("Линейное уравнение, одно решение: x = %.2f\n", x);
        }
    } else {
        // Решение квадратного уравнения
        double discriminant = b * b - 4 * a * c;

        if (discriminant > 0) {
            double x1 = (-b + sqrt(discriminant)) / (2 * a);
            double x2 = (-b - sqrt(discriminant)) / (2 * a);
            printf("Два решения: x1 = %.2f, x2 = %.2f\n", x1, x2);
        } else if (discriminant == 0) {
            double x = -b / (2 * a);
            printf("Одно решение: x = %.2f\n", x);
        } else {
            printf("Действительных решений нет.\n");
        }
    }
}

// Основная функция
int main() {
    double a, b, c;

    printf("Введите коэффициенты a, b, c: ");
    if (scanf("%lf %lf %lf", &a, &b, &c) != 3) {
        printf("Ошибка ввода.\n");
        return 1;
    }

    solve_quadratic(a, b, c);

    return 0;
}
