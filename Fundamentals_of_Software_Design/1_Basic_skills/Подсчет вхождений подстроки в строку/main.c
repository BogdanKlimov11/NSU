#include <stdio.h>
#include <string.h>

int substring_count(char *string, char *substring) {
    int count = 0;
    int sub_len = strlen(substring);

    if (sub_len == 0) return 0; // Если подстрока пустая, возвращаем 0

    char *ptr = string;
    while ((ptr = strstr(ptr, substring)) != NULL) { // Ищем вхождение подстроки
        count++;
        ptr++; // Смещаем указатель, чтобы находить пересекающиеся вхождения
    }

    return count;
}

int main() {
    printf("%d\n", substring_count("abcabc", "ab")); // 2
    printf("%d\n", substring_count("abcabcd", "d")); // 1
    printf("%d\n", substring_count("abcabcd", "q")); // 0
    printf("%d\n", substring_count("aaaaaa", "aa")); // 5
    return 0;
}
