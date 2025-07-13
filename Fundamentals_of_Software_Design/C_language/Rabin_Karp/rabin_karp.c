#include <stdlib.h>
#include <string.h>

#include "rabin_karp.h"

#define BASE 256
#define MODULUS 101

static unsigned int hash(const char* str, size_t len) {
    unsigned int h = 0;
    for (size_t i = 0; i < len; i++) {
        h = (h * BASE + str[i]) % MODULUS;
    }
    return h;
}

static unsigned int update_hash(unsigned int prev_hash, char prev_char, char new_char, size_t len) {
    unsigned int h = (prev_hash + MODULUS - (prev_char * (unsigned int)pow(BASE, len-1) % MODULUS) % MODULUS;
    return (h * BASE + new_char) % MODULUS;
}

const char* my_strstr(const char* haystack, const char* needle) {
    if (!haystack || !needle) return NULL;
    
    size_t n = strlen(haystack);
    size_t m = strlen(needle);
    
    if (m == 0) return haystack;
    if (n < m) return NULL;
    
    unsigned int needle_hash = hash(needle, m);
    unsigned int window_hash = hash(haystack, m);
    
    for (size_t i = 0; i <= n - m; i++) {
        if (window_hash == needle_hash && 
            memcmp(haystack + i, needle, m) == 0) {
            return haystack + i;
        }
        
        if (i < n - m) {
            window_hash = update_hash(window_hash, 
                                    haystack[i], 
                                    haystack[i + m], 
                                    m);
        }
    }
    
    return NULL;
}
