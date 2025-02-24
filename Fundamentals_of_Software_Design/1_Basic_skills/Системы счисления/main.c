#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Функция для преобразования символа в его числовое значение (для систем счисления от 2 до 36)
int char_to_digit(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'a' && c <= 'z') {
        return c - 'a' + 10;
    } else if (c >= 'A' && c <= 'Z') {
        return c - 'A' + 10;
    }
    return -1;  // Неверный символ
}

// Функция для преобразования числа из одной системы счисления в другую
char *transform(const char *input, int in_base, int out_base) {
    // Проверка на допустимость оснований
    if (in_base < 2 || in_base > 36 || out_base < 2 || out_base > 36) {
        return NULL;  // Ошибка: базы должны быть от 2 до 36
    }

    // Преобразование из входной системы счисления в десятичную
    long long decimal_value = 0;
    size_t length = strlen(input);
    for (size_t i = 0; i < length; i++) {
        int digit = char_to_digit(input[i]);
        if (digit < 0 || digit >= in_base) {
            return NULL;  // Ошибка: недопустимый символ для данной системы счисления
        }
        decimal_value = decimal_value * in_base + digit;
    }

    // Если число равно нулю, сразу возвращаем "0"
    if (decimal_value == 0) {
        char *result = malloc(2 * sizeof(char));  // "0" + '\0'
        if (result) {
            strcpy(result, "0");
        }
        return result;
    }

    // Преобразование из десятичной системы в выходную систему счисления
    char result[128];  // Предположим, что число не превысит 128 символов
    int index = 0;
    while (decimal_value > 0) {
        int remainder = decimal_value % out_base;
        result[index++] = (remainder < 10) ? '0' + remainder : 'a' + (remainder - 10);
        decimal_value /= out_base;
    }

    // Инвертируем результат
    result[index] = '\0';
    char *final_result = malloc((index + 1) * sizeof(char));
    if (!final_result) {
        return NULL;  // Ошибка выделения памяти
    }

    for (int i = 0; i < index; i++) {
        final_result[i] = result[index - i - 1];
    }
    final_result[index] = '\0';

    return final_result;
}

// Пример использования
int main() {
    const char *input = "prevedmedved36";  // Исходное число
    int in_base = 36;                     // Исходная система счисления
    int out_base = 2;                     // Целевая система счисления

    char *result = transform(input, in_base, out_base);
    if (result) {
        printf("Результат: %s\n", result);
        free(result);  // Освобождение памяти
    } else {
        printf("Ошибка преобразования.\n");
    }

    return 0;
}
