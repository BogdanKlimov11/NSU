#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#define STRING_LENGTH 10 // Длина строки

// Функция для генерации случайной строки
void generate_random_string(char *str, int length) {
    const char charset[] = "abcdefghijklmnopqrstuvwxyz"
                           "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                           "0123456789"
                           "!@#$%^&*()-_=+[]{};:'\",.<>?/\\|";
    int charset_size = sizeof(charset) - 1;

    for (int i = 0; i < length; i++) {
        str[i] = charset[rand() % charset_size];
    }
    str[length] = '\0'; // Завершающий символ
}

int main() {
    char random_string[STRING_LENGTH + 1];
    char user_input[STRING_LENGTH + 1];
    int index = 0;

    // Инициализация генератора случайных чисел
    srand(time(NULL));

    // Генерация случайной строки
    generate_random_string(random_string, STRING_LENGTH);

    printf("Введите следующую строку: %s\n", random_string);

    // Цикл ввода
    while (index < STRING_LENGTH) {
        printf("Введите символ '%c': ", random_string[index]);
        fgets(user_input, sizeof(user_input), stdin); // Чтение строки
        if (user_input[0] == random_string[index]) {
            index++;
        } else {
            printf("Неправильный символ. Попробуйте снова.\n");
        }
    }

    printf("Поздравляем! Вы правильно ввели строку: %s\n", random_string);

    return 0;
}
