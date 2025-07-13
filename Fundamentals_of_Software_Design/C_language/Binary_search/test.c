#include <assert.h>
#include <stdio.h>

#include "binary_search.h"

void test_empty_array() {
    int arr[] = {};
    int result = binary_search(arr, 0, 5);
    assert(result == -1 && "Empty array should return -1");
    printf("Test empty array passed\n");
}

void test_single_element() {
    int arr1[] = {5};
    int result1 = binary_search(arr1, 1, 5);
    assert(result1 == 0 && "Single element found should return 0");
    
    int arr2[] = {3};
    int result2 = binary_search(arr2, 1, 5);
    assert(result2 == -1 && "Single element not found should return -1");
    printf("Test single element passed\n");
}

void test_even_length() {
    int arr[] = {1, 3, 5, 7};
    int result1 = binary_search(arr, 4, 3);
    assert(result1 == 1 && "Element in first half should be found");
    
    int result2 = binary_search(arr, 4, 5);
    assert(result2 == 2 && "Element in second half should be found");
    
    int result3 = binary_search(arr, 4, 4);
    assert(result3 == -1 && "Missing element should return -1");
    printf("Test even length passed\n");
}

void test_odd_length() {
    int arr[] = {2, 4, 6, 8, 10};
    int result1 = binary_search(arr, 5, 4);
    assert(result1 == 1 && "Element in first half should be found");
    
    int result2 = binary_search(arr, 5, 8);
    assert(result2 == 3 && "Element in second half should be found");
    
    int result3 = binary_search(arr, 5, 7);
    assert(result3 == -1 && "Missing element should return -1");
    printf("Test odd length passed\n");
}

void test_duplicates() {
    int arr[] = {1, 2, 2, 2, 3};
    int result = binary_search(arr, 5, 2);
    assert(result >= 1 && result <= 3 && "Duplicate element should return valid index");
    printf("Test duplicates passed\n");
}

void test_large_array() {
    const size_t size = 1000000;
    int* arr = malloc(size * sizeof(int));
    if (arr == NULL) return;
    
    for (size_t i = 0; i < size; i++) {
        arr[i] = i * 2;
    }
    
    int result1 = binary_search(arr, size, 0);
    assert(result1 == 0 && "First element should be found");
    
    int result2 = binary_search(arr, size, 1999998);
    assert(result2 == 999999 && "Last element should be found");
    
    int result3 = binary_search(arr, size, 123456);
    assert(result3 == 61728 && "Middle element should be found");
    
    int result4 = binary_search(arr, size, 123457);
    assert(result4 == -1 && "Missing element should return -1");
    
    free(arr);
    printf("Test large array passed\n");
}

int main() {
    test_empty_array();
    test_single_element();
    test_even_length();
    test_odd_length();
    test_duplicates();
    test_large_array();
    printf("All binary search tests passed successfully!\n");
    return 0;
}
