#include <stdio.h>
#include <string.h>

void compute_lps(const char *needle, int m, int *lps) {
    int length = 0, i = 1;
    lps[0] = 0;

    while (i < m) {
        if (needle[i] == needle[length]) {
            lps[i++] = ++length;
        } else if (length) {
            length = lps[length - 1];
        } else {
            lps[i++] = 0;
        }
    }
}

char *my_strstr(char *haystack, const char *needle) {
    if (!*needle) return haystack; // Если needle пустая, вернуть haystack

    int n = strlen(haystack), m = strlen(needle);
    if (m > n) return NULL;

    int lps[m];
    compute_lps(needle, m, lps);

    int i = 0, j = 0;
    while (i < n) {
        if (haystack[i] == needle[j]) {
            i++, j++;
            if (j == m) return haystack + i - j;
        } else {
            j = j ? lps[j - 1] : 0;
            if (!j) i++;
        }
    }
    return NULL;
}

// --- Тесты ---
void run_tests() {
    struct { char *haystack; char *needle; int expected; } tests[] = {
        {"hello", "ll", 2},
        {"abcdef", "cd", 2},
        {"aaaaa", "bba", -1},
        {"mississippi", "issi", 1},
        {"abc", "", 0},
        {"", "abc", -1},
        {"abc", "abc", 0}
    };
    
    int num_tests = sizeof(tests) / sizeof(tests[0]);
    for (int i = 0; i < num_tests; i++) {
        char *result = my_strstr(tests[i].haystack, tests[i].needle);
        int pos = result ? result - tests[i].haystack : -1;
        printf("Test %d: %s\n", i + 1, pos == tests[i].expected ? "+" : "-");
    }
}

int main() {
    run_tests();
    return 0;
}
