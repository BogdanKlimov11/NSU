#ifndef AHO_CORASICK_H
#define AHO_CORASICK_H

#include <stddef.h>

typedef struct {
    int* positions;
    size_t count;
} SearchResult;

SearchResult substring_search(const char* text, const char** patterns, size_t pattern_count);

#endif
