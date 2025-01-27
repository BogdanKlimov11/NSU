#include <stdio.h>
#include <ctype.h>
#include <string.h>

void strtrim(char *string) {
    if (!string || *string == '\0') return;

    // Указатель на начало строки без начальных пробелов
    char *start = string;
    while (isspace((unsigned char)*start)) {
        start++;
    }

    // Указатель на конец строки без конечных пробелов
    char *end = string + strlen(string) - 1;
    while (end > start && isspace((unsigned char)*end)) {
        *end = '\0'; // Обнуляем символы пробелов
        end--;
    }

    // Сдвигаем оставшуюся часть строки к началу
    if (start != string) {
        memmove(string, start, end - start + 2);
    }
}

int main() {
    char buf[] = "   abc   ";
    printf("<%s>\n", buf); // <   abc   >
    strtrim(buf);
    printf("<%s>\n", buf); // <abc>
    return 0;
}
