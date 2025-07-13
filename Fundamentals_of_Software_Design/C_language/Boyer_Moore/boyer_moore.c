#include <limits.h>
#include <string.h>

#include "boyer_moore.h"

#define ALPHABET_SIZE 256

static void preprocess_bad_char(const char* needle, size_t len, int bad_char[ALPHABET_SIZE]) {
    for (size_t i = 0; i < ALPHABET_SIZE; i++) {
        bad_char[i] = -1;
    }
    for (size_t i = 0; i < len; i++) {
        bad_char[(unsigned char)needle[i]] = i;
    }
}

static void preprocess_good_suffix(const char* needle, size_t len, int* good_suffix) {
    int* suffix = (int*)malloc(len * sizeof(int));
    if (!suffix) return;

    suffix[len-1] = len;
    for (int i = len-2; i >= 0; i--) {
        int j = i;
        while (j >= 0 && needle[j] == needle[len-1 - (i-j)]) {
            j--;
        }
        suffix[i] = i - j;
    }

    for (size_t i = 0; i < len; i++) {
        good_suffix[i] = len;
    }

    for (int i = len-1; i >= 0; i--) {
        if (suffix[i] == i+1) {
            for (size_t j = 0; j < len-1 - i; j++) {
                if (good_suffix[j] == (int)len) {
                    good_suffix[j] = len-1 - i;
                }
            }
        }
    }

    for (size_t i = 0; i < len-1; i++) {
        good_suffix[len-1 - suffix[i]] = len-1 - i;
    }

    free(suffix);
}

const char* boyer_moore_search(const char* haystack, const char* needle) {
    if (!haystack || !needle) return NULL;
    
    size_t n = strlen(haystack);
    size_t m = strlen(needle);
    
    if (m == 0) return haystack;
    if (n < m) return NULL;

    int bad_char[ALPHABET_SIZE];
    preprocess_bad_char(needle, m, bad_char);

    int* good_suffix = (int*)malloc(m * sizeof(int));
    if (!good_suffix) return NULL;
    preprocess_good_suffix(needle, m, good_suffix);

    size_t s = 0;
    while (s <= n - m) {
        int j = m - 1;
        while (j >= 0 && needle[j] == haystack[s + j]) {
            j--;
        }
        
        if (j < 0) {
            free(good_suffix);
            return haystack + s;
        } else {
            int bc_shift = j - bad_char[(unsigned char)haystack[s + j]];
            int gs_shift = good_suffix[j];
            s += bc_shift > gs_shift ? bc_shift : gs_shift;
        }
    }

    free(good_suffix);
    return NULL;
}
