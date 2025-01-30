#include <stdio.h>
#include <stdlib.h>

int is_lucky_ticket(char* ticket) {
    int sum1 = 0, sum2 = 0;

    // Считаем сумму первых 16 цифр
    for (int i = 0; i < 16; i++) {
        sum1 += ticket[i] - '0';  // Преобразуем символ в цифру
    }

    // Считаем сумму последних 16 цифр
    for (int i = 16; i < 32; i++) {
        sum2 += ticket[i] - '0';  // Преобразуем символ в цифру
    }

    return sum1 == sum2;  // Если суммы равны, билет счастливый
}

int main() {
    int total_tickets = 1000000;  // Количество билетов для проверки
    int lucky_count = 0;  // Количество счастливых билетов

    // Генерируем случайные билеты
    for (int i = 0; i < total_tickets; i++) {
        // Генерируем случайный 32-значный номер билета
        char ticket[33];  // Массив для хранения билета
        for (int j = 0; j < 32; j++) {
            ticket[j] = '0' + (rand() % 10);  // Случайная цифра от 0 до 9
        }
        ticket[32] = '\0';  // Завершающий ноль для строки

        // Проверяем, является ли билет счастливым
        if (is_lucky_ticket(ticket)) {
            lucky_count++;
        }
    }

    // Вычисляем долю счастливых билетов
    double lucky_percentage = (double)lucky_count / total_tickets * 100;

    printf("Доля счастливых билетов: %.6f%%\n", lucky_percentage);

    return 0;
}
