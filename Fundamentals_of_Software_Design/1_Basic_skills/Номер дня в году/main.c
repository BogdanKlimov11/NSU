#include <stdio.h>

int is_leap_year(int year) {
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

int day_number(int d, int m, int y) {
    if (m < 1 || m > 12 || d < 1) return -1; // Проверка корректности месяца и дня

    int days_in_months[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (is_leap_year(y)) days_in_months[1] = 29; // Коррекция февраля для високосного года

    if (d > days_in_months[m - 1]) return -1; // Проверка корректности дня

    int day_count = d;
    for (int i = 0; i < m - 1; i++) {
        day_count += days_in_months[i];
    }
    
    return day_count;
}

int main() {
    int d, m, y;
    printf("Введите день, месяц и год: ");
    scanf("%d %d %d", &d, &m, &y);

    int result = day_number(d, m, y);
    if (result == -1)
        printf("Некорректная дата\n");
    else
        printf("Номер дня в году: %d\n", result);

    return 0;
}
