#include <stdio.h>
#include <ctype.h>
#include <assert.h>

int parse_number(char *string, int base, int *is_ok) {
    // Пропускаем пробельные символы в начале строки
    while (*string && isspace((unsigned char)*string)) {
        string++;
    }

    // Проверяем на наличие знака (по умолчанию +)
    int sign = 1;
    if (*string == '-') {
        sign = -1;
        string++;
    } else if (*string == '+') {
        string++;
    }

    // Проверка корректности базы
    if (base < 2 || base > 36) {
        *is_ok = 0;
        return 0;
    }

    int result = 0;
    int valid = 0;

    // Проходим по строке и преобразуем символы в числа
    while (*string) {
        char c = *string++;
        int digit = 0;

        // Определяем значение цифры
        if (isdigit(c)) {
            digit = c - '0';  // цифры от 0 до 9
        } else if (isalpha(c)) {
            digit = tolower(c) - 'a' + 10;  // буквы от a до z
        } else {
            // Если символ не является допустимым для текущей системы счисления
            break;
        }

        // Проверяем, что цифра входит в допустимый диапазон для указанной базы
        if (digit >= base) {
            break;
        }

        // Добавляем цифру в результат
        result = result * base + digit;
        valid = 1;
    }

    // Если мы не прочитали ни одной цифры, считаем, что ошибка
    if (!valid) {
        *is_ok = 0;
        return 0;
    }

    // Устанавливаем флаг успеха и возвращаем результат с учетом знака
    *is_ok = 1;
    return sign * result;
}

int main() {
    int is_ok;

    // Тесты
    assert(parse_number("123", 10, &is_ok) == 123 && is_ok == 1);
    assert(parse_number("1010", 2, &is_ok) == 10 && is_ok == 1);
    assert(parse_number("1A", 16, &is_ok) == 26 && is_ok == 1);
    assert(parse_number("Z", 36, &is_ok) == 35 && is_ok == 1);
    assert(parse_number("  10", 10, &is_ok) == 10 && is_ok == 1);
    assert(parse_number("-FF", 16, &is_ok) == -255 && is_ok == 1);
    assert(parse_number("G", 16, &is_ok) == 0 && is_ok == 0);  // Неверная цифра для базы 16

    // Печать для проверки
    printf("Тесты пройдены.\n");

    return 0;
}
