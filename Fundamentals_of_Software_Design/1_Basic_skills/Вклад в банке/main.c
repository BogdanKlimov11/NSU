#include <stdio.h>
#include <math.h>

void calculate_deposit(double amount, int months, double annual_rate) {
    double monthly_rate = annual_rate / 12 / 100;
    double simple_interest = amount;
    double compound_interest = amount;

    printf("%-6s %-15s %-15s\n", "Месяц", "Простые %", "Сложные %");
    printf("----------------------------------------\n");

    for (int month = 1; month <= months; month++) {
        simple_interest += amount * (annual_rate / 100) / 12;
        compound_interest *= (1 + monthly_rate);
        printf("%-6d %-15.2f %-15.2f\n", month, simple_interest, compound_interest);
    }
}

int main() {
    double amount, annual_rate;
    int months;

    printf("Введите сумму вклада: ");
    scanf("%lf", &amount);
    printf("Введите срок вклада (в месяцах): ");
    scanf("%d", &months);
    printf("Введите годовой процент: ");
    scanf("%lf", &annual_rate);

    calculate_deposit(amount, months, annual_rate);

    return 0;
}
