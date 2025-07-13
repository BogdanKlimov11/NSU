#include <stdlib.h>
#include <string.h>

#include "z_function.h"

static int* compute_z_array(const char* str, size_t len) {
    int* Z = (int*)malloc(len * sizeof(int));
    if (!Z) return NULL;
    
    Z[0] = len;
    int l = 0, r = 0;
    
    for (int i = 1; i < len; i++) {
        if (i > r) {
            l = r = i;
            while (r < len && str[r - l] == str[r]) {
                r++;
            }
            Z[i] = r - l;
            r--;
        } else {
            int k = i - l;
            if (Z[k] < r - i + 1) {
                Z[i] = Z[k];
            } else {
                l = i;
                while (r < len && str[r - l] == str[r]) {
                    r++;
                }
                Z[i] = r - l;
                r--;
            }
        }
    }
    return Z;
}

int* substring_search(const char* text, const char* pattern, size_t* result_count) {
    if (!text || !pattern) {
        *result_count = 0;
        return NULL;
    }
    
    size_t text_len = strlen(text);
    size_t pattern_len = strlen(pattern);
    
    if (pattern_len == 0) {
        *result_count = text_len + 1;
        int* result = (int*)malloc((text_len + 1) * sizeof(int));
        for (size_t i = 0; i <= text_len; i++) {
            result[i] = i;
        }
        return result;
    }
    
    size_t concat_len = pattern_len + 1 + text_len;
    char* concat = (char*)malloc(concat_len + 1);
    if (!concat) {
        *result_count = 0;
        return NULL;
    }
    
    strcpy(concat, pattern);
    concat[pattern_len] = '$';
    strcpy(concat + pattern_len + 1, text);
    
    int* Z = compute_z_array(concat, concat_len);
    free(concat);
    
    if (!Z) {
        *result_count = 0;
        return NULL;
    }
    
    size_t count = 0;
    for (size_t i = pattern_len + 1; i < concat_len; i++) {
        if (Z[i] == pattern_len) {
            count++;
        }
    }
    
    int* result = (int*)malloc(count * sizeof(int));
    if (!result) {
        free(Z);
        *result_count = 0;
        return NULL;
    }
    
    size_t idx = 0;
    for (size_t i = pattern_len + 1; i < concat_len; i++) {
        if (Z[i] == pattern_len) {
            result[idx++] = i - pattern_len - 1;
        }
    }
    
    free(Z);
    *result_count = count;
    return result;
}
