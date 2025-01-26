#include <stdio.h>

double celsius_to_fahrenheit(double celsius) {
    return celsius * 9.0 / 5.0 + 32.0;
}

double fahrenheit_to_celsius(double fahrenheit) {
    return (fahrenheit - 32.0) * 5.0 / 9.0;
}

int main() {
    int choice;
    double temperature, converted;

    printf("Выберите режим перевода:\n");
    printf("1. Цельсий в Фаренгейт\n");
    printf("2. Фаренгейт в Цельсий\n");
    printf("Введите номер режима (1 или 2): ");
    scanf("%d", &choice);

    if (choice == 1) {
        printf("Введите температуру в градусах Цельсия: ");
        scanf("%lf", &temperature);
        converted = celsius_to_fahrenheit(temperature);
        printf("%.2f°C = %.2f°F\n", temperature, converted);
    } else if (choice == 2) {
        printf("Введите температуру в градусах Фаренгейта: ");
        scanf("%lf", &temperature);
        converted = fahrenheit_to_celsius(temperature);
        printf("%.2f°F = %.2f°C\n", temperature, converted);
    } else {
        printf("Некорректный выбор режима.\n");
    }

    return 0;
}
