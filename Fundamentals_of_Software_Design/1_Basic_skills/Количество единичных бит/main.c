#include <stdio.h>

// Функция для подсчета количества бит, установленных в 1
int count_set_bits(int num) {
    int count = 0;

    while (num != 0) {
        count += num & 1; // Проверяем младший бит
        num >>= 1;        // Сдвигаем число вправо на 1 бит
    }

    return count;
}

int main() {
    int num;

    printf("Введите число: ");
    scanf("%d", &num);

    int result = count_set_bits(num);

    printf("Количество единичных бит в числе %d: %d\n", num, result);

    return 0;
}
