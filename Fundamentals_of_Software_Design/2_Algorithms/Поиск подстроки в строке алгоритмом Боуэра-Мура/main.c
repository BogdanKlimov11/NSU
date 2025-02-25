#include <stdio.h>
#include <string.h>

#define ALPHABET_SIZE 256  // Размер алфавита (ASCII)

void preprocess_bad_character_rule(const char *needle, int *bad_char_table) {
    int m = strlen(needle);

    // Инициализируем таблицу плохих символов (-1 означает, что символ не найден в паттерне)
    for (int i = 0; i < ALPHABET_SIZE; i++) {
        bad_char_table[i] = -1;
    }

    // Заполняем таблицу плохих символов
    for (int i = 0; i < m; i++) {
        bad_char_table[(unsigned char)needle[i]] = i;
    }
}

int my_strstr(const char *haystack, const char *needle) {
    int n = strlen(haystack);
    int m = strlen(needle);

    if (m == 0) {
        return 0;  // Пустая строка всегда найдется в любой строке
    }

    int bad_char_table[ALPHABET_SIZE];
    preprocess_bad_character_rule(needle, bad_char_table);

    int i = 0;
    while (i <= n - m) {
        int j = m - 1;

        // Сравниваем подстроку haystack с needle, начиная с конца needle
        while (j >= 0 && haystack[i + j] == needle[j]) {
            j--;
        }

        // Если нашли полное совпадение
        if (j < 0) {
            return i;  // Возвращаем индекс начала совпадения
        }

        // Сдвигаем паттерн в соответствии с таблицей плохих символов
        i += (j - bad_char_table[(unsigned char)haystack[i + j]] > 0) ?
             j - bad_char_table[(unsigned char)haystack[i + j]] : 1;
    }

    return -1;  // Если подстрока не найдена
}

int main() {
    const char *haystack = "this is a simple example";
    const char *needle = "simple";

    int result = my_strstr(haystack, needle);

    if (result != -1) {
        printf("Подстрока найдена на позиции: %d\n", result);
    } else {
        printf("Подстрока не найдена\n");
    }

    return 0;
}
