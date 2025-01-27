#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

double estimate_pi(int num_points) {
    int inside_circle = 0;

    for (int i = 0; i < num_points; i++) {
        // Генерация случайных координат (x, y) в диапазоне [0, 1]
        double x = (double)rand() / RAND_MAX;
        double y = (double)rand() / RAND_MAX;

        // Проверяем, попадает ли точка в четверть окружности
        if (x * x + y * y <= 1.0) {
            inside_circle++;
        }
    }

    // Вычисление числа π
    return (4.0 * inside_circle) / num_points;
}

int main() {
    int num_points;

    // Инициализация генератора случайных чисел
    srand((unsigned int)time(NULL));

    // Ввод количества точек
    printf("Введите количество точек для метода Монте-Карло: ");
    scanf("%d", &num_points);

    if (num_points <= 0) {
        printf("Количество точек должно быть положительным.\n");
        return 1;
    }

    // Оценка числа π
    double pi_estimate = estimate_pi(num_points);
    printf("Оценка числа π: %.6f\n", pi_estimate);

    return 0;
}
