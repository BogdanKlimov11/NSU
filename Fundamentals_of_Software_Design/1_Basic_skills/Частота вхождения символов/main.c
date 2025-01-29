#include <stdio.h>

void char_frequency(const char *s) {
    int freq[256] = {0}; // Массив для подсчёта символов (ASCII 0-255)

    // Подсчет количества вхождений символов
    for (int i = 0; s[i] != '\0'; i++) {
        freq[(unsigned char)s[i]]++;
    }

    // Вывод результатов
    for (int i = 0; i < 256; i++) {
        if (freq[i] > 0) {
            printf("\"%c\" — %d раз(а).\n", i, freq[i]);
        }
    }
}

int main() {
    char s[] = "aaaabccc";
    char_frequency(s);
    return 0;
}
