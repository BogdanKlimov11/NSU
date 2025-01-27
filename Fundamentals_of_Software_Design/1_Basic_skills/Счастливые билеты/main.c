#include <stdio.h>

int main() {
    int total_tickets = 0;         // Общее количество билетов
    int lucky_tickets = 0;         // Количество счастливых билетов

    // Перебираем все 6-значные номера билетов
    for (int ticket = 0; ticket <= 999999; ticket++) {
        // Разделяем номер билета на цифры
        int a = (ticket / 100000) % 10; // Первая цифра
        int b = (ticket / 10000) % 10;  // Вторая цифра
        int c = (ticket / 1000) % 10;   // Третья цифра
        int d = (ticket / 100) % 10;    // Четвёртая цифра
        int e = (ticket / 10) % 10;     // Пятая цифра
        int f = ticket % 10;            // Шестая цифра

        // Сравниваем суммы первых и последних трёх цифр
        if (a + b + c == d + e + f) {
            lucky_tickets++;
        }

        total_tickets++;
    }

    // Вычисляем процент счастливых билетов
    double percentage = (double)lucky_tickets / total_tickets * 100;

    // Вывод результатов
    printf("Общее количество билетов: %d\n", total_tickets);
    printf("Количество счастливых билетов: %d\n", lucky_tickets);
    printf("Процент счастливых билетов: %.2f%%\n", percentage);

    return 0;
}
