#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "boyer_moore.h"

void test_empty_needle() {
    const char* result = my_strstr("abc", "");
    assert(result != NULL && strcmp(result, "abc") == 0);
    printf("Test empty needle passed\n");
}

void test_empty_haystack() {
    const char* result = my_strstr("", "abc");
    assert(result == NULL);
    printf("Test empty haystack passed\n");
}

void test_needle_longer() {
    const char* result = my_strstr("short", "very long needle");
    assert(result == NULL);
    printf("Test needle longer passed\n");
}

void test_exact_match() {
    const char* result = my_strstr("hello world", "hello world");
    assert(result != NULL && strcmp(result, "hello world") == 0);
    printf("Test exact match passed\n");
}

void test_substring_at_start() {
    const char* result = my_strstr("hello world", "hello");
    assert(result != NULL && strcmp(result, "hello world") == 0);
    printf("Test substring at start passed\n");
}

void test_substring_at_end() {
    const char* result = boyer_moore_search("hello world", "world");
    assert(result != NULL && strcmp(result, "world") == 0);
    printf("Test substring at end passed\n");
}

void test_substring_in_middle() {
    const char* result = my_strstr("hello beautiful world", "beautiful");
    assert(result != NULL && strcmp(result, "beautiful world") == 0);
    printf("Test substring in middle passed\n");
}

void test_multiple_occurrences() {
    const char* result = my_strstr("abababab", "aba");
    assert(result != NULL && strcmp(result, "abababab") == 0);
    printf("Test multiple occurrences passed\n");
}

void test_no_match() {
    const char* result = my_strstr("hello world", "test");
    assert(result == NULL);
    printf("Test no match passed\n");
}

void test_unicode() {
    const char* result = my_strstr("привет мир", "мир");
    assert(result != NULL && strcmp(result, "мир") == 0);
    printf("Test unicode passed\n");
}

void test_large_text() {
    const size_t size = 1000000;
    char* haystack = malloc(size + 1);
    if (!haystack) return;
    
    memset(haystack, 'a', size);
    haystack[size] = '\0';
    memcpy(haystack + size - 5, "needle", 6);
    
    const char* result = my_strstr(haystack, "needle");
    assert(result != NULL && strcmp(result, "needle") == 0);
    
    free(haystack);
    printf("Test large text passed\n");
}

int main() {
    test_empty_needle();
    test_empty_haystack();
    test_needle_longer();
    test_exact_match();
    test_substring_at_start();
    test_substring_at_end();
    test_substring_in_middle();
    test_multiple_occurrences();
    test_no_match();
    test_unicode();
    test_large_text();
    
    printf("All tests passed successfully!\n");
    return 0;
}
