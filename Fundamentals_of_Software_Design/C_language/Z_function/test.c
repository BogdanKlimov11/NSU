#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "z_function.h"

void test_empty_pattern() {
    size_t count;
    int* result = substring_search("abc", "", &count);
    assert(count == 4);
    assert(result[0] == 0 && result[1] == 1 && result[2] == 2 && result[3] == 3);
    free(result);
    printf("Test empty pattern passed\n");
}

void test_empty_text() {
    size_t count;
    int* result = substring_search("", "a", &count);
    assert(count == 0);
    assert(result == NULL);
    printf("Test empty text passed\n");
}

void test_single_match() {
    size_t count;
    int* result = substring_search("abcde", "bcd", &count);
    assert(count == 1);
    assert(result[0] == 1);
    free(result);
    printf("Test single match passed\n");
}

void test_multiple_matches() {
    size_t count;
    int* result = substring_search("ababab", "aba", &count);
    assert(count == 2);
    assert(result[0] == 0 && result[1] == 2);
    free(result);
    printf("Test multiple matches passed\n");
}

void test_no_match() {
    size_t count;
    int* result = substring_search("abcdef", "xyz", &count);
    assert(count == 0);
    assert(result == NULL);
    printf("Test no match passed\n");
}

void test_pattern_equals_text() {
    size_t count;
    int* result = substring_search("abc", "abc", &count);
    assert(count == 1);
    assert(result[0] == 0);
    free(result);
    printf("Test pattern equals text passed\n");
}

void test_overlapping() {
    size_t count;
    int* result = substring_search("aaa", "aa", &count);
    assert(count == 2);
    assert(result[0] == 0 && result[1] == 1);
    free(result);
    printf("Test overlapping passed\n");
}

void test_unicode() {
    size_t count;
    int* result = substring_search("привет мир", "мир", &count);
    assert(count == 1);
    assert(result[0] == 7);
    free(result);
    printf("Test unicode passed\n");
}

void test_large_text() {
    const size_t size = 1000000;
    char* text = malloc(size + 1);
    memset(text, 'a', size);
    text[size] = '\0';
    memcpy(text + size - 5, "needle", 6);
    
    size_t count;
    int* result = substring_search(text, "needle", &count);
    assert(count == 1);
    assert(result[0] == size - 5);
    free(result);
    free(text);
    printf("Test large text passed\n");
}

int main() {
    test_empty_pattern();
    test_empty_text();
    test_single_match();
    test_multiple_matches();
    test_no_match();
    test_pattern_equals_text();
    test_overlapping();
    test_unicode();
    test_large_text();
    
    printf("All tests passed successfully!\n");
    return 0;
}
