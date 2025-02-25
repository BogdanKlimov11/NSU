#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void z_function(const char *s, int *z, int n) {
    int l = 0, r = 0;
    for (int i = 1; i < n; i++) {
        if (i <= r)
            z[i] = (r - i + 1 < z[i - l]) ? r - i + 1 : z[i - l];
        while (i + z[i] < n && s[z[i]] == s[i + z[i]])
            z[i]++;
        if (i + z[i] - 1 > r) {
            l = i;
            r = i + z[i] - 1;
        }
    }
}

void substring_search(const char *text, const char *pattern) {
    int text_len = strlen(text);
    int pattern_len = strlen(pattern);
    int total_len = pattern_len + text_len + 1;
    
    char *combined = (char *)malloc(total_len + 1);
    sprintf(combined, "%s#%s", pattern, text);
    
    int *z = (int *)calloc(total_len, sizeof(int));
    z_function(combined, z, total_len);

    for (int i = pattern_len + 1; i < total_len; i++) {
        if (z[i] == pattern_len) {
            printf("%d ", i - pattern_len - 1);
        }
    }
    printf("\n");

    free(combined);
    free(z);
}

// Тесты
void test() {
    printf("Тест 1: "); substring_search("ababcababc", "abc"); // Ожидается: 2 7
    printf("Тест 2: "); substring_search("aaaaa", "aa");       // Ожидается: 0 1 2 3
    printf("Тест 3: "); substring_search("abcdef", "xyz");     // Ожидается: (ничего)
    printf("Тест 4: "); substring_search("abababab", "aba");   // Ожидается: 0 2 4
    printf("Тест 5: "); substring_search("", "a");             // Ожидается: (ничего)
    printf("Тест 6: "); substring_search("a", "a");            // Ожидается: 0
}

int main() {
    test();
    return 0;
}
