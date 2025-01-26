#include <iostream>
#include <iomanip> // Для настройки вывода с фиксированным количеством знаков после запятой

double celsius_to_fahrenheit(double celsius) {
    return celsius * 9.0 / 5.0 + 32.0;
}

double fahrenheit_to_celsius(double fahrenheit) {
    return (fahrenheit - 32.0) * 5.0 / 9.0;
}

int main() {
    int choice;
    double temperature, converted;

    std::cout << "Выберите режим перевода:\n";
    std::cout << "1. Цельсий в Фаренгейт\n";
    std::cout << "2. Фаренгейт в Цельсий\n";
    std::cout << "Введите номер режима (1 или 2): ";
    std::cin >> choice;

    if (choice == 1) {
        std::cout << "Введите температуру в градусах Цельсия: ";
        std::cin >> temperature;
        converted = celsius_to_fahrenheit(temperature);
        std::cout << std::fixed << std::setprecision(2);
        std::cout << temperature << "°C = " << converted << "°F\n";
    } else if (choice == 2) {
        std::cout << "Введите температуру в градусах Фаренгейта: ";
        std::cin >> temperature;
        converted = fahrenheit_to_celsius(temperature);
        std::cout << std::fixed << std::setprecision(2);
        std::cout << temperature << "°F = " << converted << "°C\n";
    } else {
        std::cout << "Некорректный выбор режима.\n";
    }

    return 0;
}
