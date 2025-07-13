#include <stdlib.h>
#include <string.h>

#include "knuth_morris_pratt.h"

static int* compute_lps(const char* needle, size_t len) {
    int* lps = (int*)malloc(len * sizeof(int));
    if (!lps) return NULL;
    
    lps[0] = 0;
    size_t i = 1;
    size_t j = 0;
    
    while (i < len) {
        if (needle[i] == needle[j]) {
            lps[i] = j + 1;
            i++;
            j++;
        } else {
            if (j != 0) {
                j = lps[j - 1];
            } else {
                lps[i] = 0;
                i++;
            }
        }
    }
    
    return lps;
}

const char* my_strstr(const char* haystack, const char* needle) {
    if (!haystack || !needle) return NULL;
    
    size_t n = strlen(haystack);
    size_t m = strlen(needle);
    
    if (m == 0) return haystack;
    if (n < m) return NULL;
    
    int* lps = compute_lps(needle, m);
    if (!lps) return NULL;
    
    size_t i = 0;
    size_t j = 0;
    
    while (i < n) {
        if (haystack[i] == needle[j]) {
            i++;
            j++;
            
            if (j == m) {
                free(lps);
                return haystack + (i - j);
            }
        } else {
            if (j != 0) {
                j = lps[j - 1];
            } else {
                i++;
            }
        }
    }
    
    free(lps);
    return NULL;
}
