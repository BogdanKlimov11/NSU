#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aho_corasick.h"

void test_empty_text() {
    const char* patterns[] = {"abc"};
    SearchResult result = substring_search("", patterns, 1);
    assert(result.count == 0);
    free(result.positions);
    printf("Test empty text passed\n");
}

void test_empty_patterns() {
    const char* patterns[] = {NULL};
    SearchResult result = substring_search("abc", patterns, 1);
    assert(result.count == 0);
    free(result.positions);
    printf("Test empty patterns passed\n");
}

void test_single_pattern() {
    const char* patterns[] = {"abc"};
    SearchResult result = substring_search("xyzabcdef", patterns, 1);
    assert(result.count == 1);
    assert(result.positions[0] == 3);
    free(result.positions);
    printf("Test single pattern passed\n");
}

void test_multiple_patterns() {
    const char* patterns[] = {"abc", "def", "ghi"};
    SearchResult result = substring_search("abcxyzdef123ghi", patterns, 3);
    assert(result.count == 3);
    assert(result.positions[0] == 0);
    assert(result.positions[1] == 6);
    assert(result.positions[2] == 12);
    free(result.positions);
    printf("Test multiple patterns passed\n");
}

void test_overlapping_patterns() {
    const char* patterns[] = {"aa", "aaa"};
    SearchResult result = substring_search("aaaaa", patterns, 2);
    assert(result.count == 7);
    free(result.positions);
    printf("Test overlapping patterns passed\n");
}

void test_unicode() {
    const char* patterns[] = {"мир"};
    SearchResult result = substring_search("привет мир", patterns, 1);
    assert(result.count == 1);
    assert(result.positions[0] == 7);
    free(result.positions);
    printf("Test unicode passed\n");
}

void test_large_text() {
    const size_t size = 1000000;
    char* text = malloc(size + 1);
    memset(text, 'a', size);
    text[size] = '\0';
    memcpy(text + size - 10, "needle", 6);
    
    const char* patterns[] = {"needle"};
    SearchResult result = substring_search(text, patterns, 1);
    assert(result.count == 1);
    assert(result.positions[0] == size - 10);
    
    free(result.positions);
    free(text);
    printf("Test large text passed\n");
}

int main() {
    test_empty_text();
    test_empty_patterns();
    test_single_pattern();
    test_multiple_patterns();
    test_overlapping_patterns();
    test_unicode();
    test_large_text();
    
    printf("All tests passed successfully!\n");
    return 0;
}
