#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define BASE 256       // Основание для хеширования (число символов в алфавите, например, ASCII)
#define MOD 101        // Модуль для хеширования (простое число, чтобы уменьшить коллизии)

// Вычисление хеша строки длины m
unsigned long long compute_hash(const char *str, int m) {
    unsigned long long hash = 0;
    for (int i = 0; i < m; i++) {
        hash = (hash * BASE + str[i]) % MOD;
    }
    return hash;
}

// Обновление хеша при сдвиге окна на один символ
unsigned long long roll_hash(unsigned long long old_hash, char old_char, char new_char, int m) {
    old_hash = (old_hash - old_char * power(BASE, m - 1)) % MOD; // Убираем старый символ
    old_hash = (old_hash * BASE + new_char) % MOD; // Добавляем новый символ
    return (old_hash + MOD) % MOD; // Модуль для предотвращения отрицательных значений
}

// Функция поиска подстроки алгоритмом Рабина-Карпа
int my_strstr(char *haystack, const char *needle) {
    int n = strlen(haystack);
    int m = strlen(needle);
    if (m == 0) return 0;  // Пустая строка всегда найдется на первой позиции
    if (n < m) return -1;  // Если строка меньше подстроки, то совпадения нет

    unsigned long long needle_hash = compute_hash(needle, m);  // Хеш подстроки
    unsigned long long haystack_hash = compute_hash(haystack, m);  // Хеш первого окна

    // Предсчитаем степень BASE^(m-1) для использования в roll_hash
    unsigned long long base_m = 1;
    for (int i = 0; i < m - 1; i++) {
        base_m = (base_m * BASE) % MOD;
    }

    // Смотрим на каждое окно длины m в строке haystack
    for (int i = 0; i <= n - m; i++) {
        // Если хеши совпали, проверяем совпадение строк
        if (haystack_hash == needle_hash) {
            if (strncmp(haystack + i, needle, m) == 0) {
                return i;  // Совпадение найдено
            }
        }

        // Если мы не достигли конца строки, сдвигаем окно
        if (i < n - m) {
            haystack_hash = roll_hash(haystack_hash, haystack[i], haystack[i + m], m);
        }
    }

    return -1;  // Подстрока не найдена
}

// Тестирование
int main() {
    char haystack[] = "this is a simple example";
    char needle[] = "simple";

    int result = my_strstr(haystack, needle);

    if (result != -1) {
        printf("Подстрока найдена на позиции: %d\n", result);
    } else {
        printf("Подстрока не найдена\n");
    }

    return 0;
}
