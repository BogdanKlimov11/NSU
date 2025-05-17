#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "selection_sort.h"

int is_sorted(const int* arr, size_t size) {
    for (size_t i = 0; i < size - 1; i++) {
        if (arr[i] > arr[i + 1]) return 0;
    }
    return 1;
}

void test_empty_array() {
    int arr[] = {};
    int* sorted = selection_sort(arr, 0);
    assert(sorted != NULL && "Empty array should return non-NULL");
    assert(is_sorted(sorted, 0) && "Empty array should be sorted");
    free(sorted);
    printf("Test empty array passed\n");
}

void test_single_element() {
    int arr[] = {1};
    int* sorted = selection_sort(arr, 1);
    assert(sorted != NULL && "Single element array should return non-NULL");
    assert(sorted[0] == 1 && "Single element should remain unchanged");
    assert(is_sorted(sorted, 1) && "Single element array should be sorted");
    free(sorted);
    printf("Test single element passed\n");
}

void test_two_elements() {
    int arr1[] = {2, 1};
    int* sorted1 = selection_sort(arr1, 2);
    assert(sorted1 != NULL && "Two elements array should return non-NULL");
    assert(sorted1[0] == 1 && sorted1[1] == 2 && "Two elements should be sorted");
    assert(is_sorted(sorted1, 2) && "Two elements array should be sorted");
    free(sorted1);

    int arr2[] = {1, 2};
    int* sorted2 = selection_sort(arr2, 2);
    assert(sorted2 != NULL && "Sorted two elements array should return non-NULL");
    assert(sorted2[0] == 1 && sorted2[1] == 2 && "Already sorted two elements should stay sorted");
    assert(is_sorted(sorted2, 2) && "Already sorted two elements should be sorted");
    free(sorted2);
    printf("Test two elements passed\n");
}

void test_even_length() {
    int arr1[] = {4, 2, 3, 1};
    int* sorted1 = selection_sort(arr1, 4);
    assert(sorted1 != NULL && "Even length array should return non-NULL");
    assert(sorted1[0] == 1 && sorted1[1] == 2 && sorted1[2] == 3 && sorted1[3] == 4 && "Even length array should be sorted");
    assert(is_sorted(sorted1, 4) && "Even length array should be sorted");
    free(sorted1);

    int arr2[] = {1, 2, 3, 4};
    int* sorted2 = selection_sort(arr2, 4);
    assert(sorted2 != NULL && "Sorted even length array should return non-NULL");
    assert(sorted2[0] == 1 && sorted2[1] == 2 && sorted2[2] == 3 && sorted2[3] == 4 && "Already sorted even length should stay sorted");
    assert(is_sorted(sorted2, 4) && "Already sorted even length should be sorted");
    free(sorted2);
    printf("Test even length passed\n");
}

void test_odd_length() {
    int arr1[] = {3, 1, 4, 1, 5};
    int* sorted1 = selection_sort(arr1, 5);
    assert(sorted1 != NULL && "Odd length array should return non-NULL");
    assert(sorted1[0] == 1 && sorted1[1] == 1 && sorted1[2] == 3 && sorted1[3] == 4 && sorted1[4] == 5 && "Odd length array should be sorted");
    assert(is_sorted(sorted1, 5) && "Odd length array should be sorted");
    free(sorted1);

    int arr2[] = {1, 2, 3, 4, 5};
    int* sorted2 = selection_sort(arr2, 5);
    assert(sorted2 != NULL && "Sorted odd length array should return non-NULL");
    assert(sorted2[0] == 1 && sorted2[1] == 2 && sorted2[2] == 3 && sorted2[3] == 4 && sorted2[4] == 5 && "Already sorted odd length should stay sorted");
    assert(is_sorted(sorted2, 5) && "Already sorted odd length should be sorted");
    free(sorted2);
    printf("Test odd length passed\n");
}

void test_same_elements() {
    int arr1[] = {2, 2, 2, 2};
    int* sorted1 = selection_sort(arr1, 4);
    assert(sorted1 != NULL && "Same elements array should return non-NULL");
    assert(sorted1[0] == 2 && sorted1[1] == 2 && sorted1[2] == 2 && sorted1[3] == 2 && "Array with same elements should remain unchanged");
    assert(is_sorted(sorted1, 4) && "Array with same elements should be sorted");
    free(sorted1);

    int arr2[] = {1, 1, 1};
    int* sorted2 = selection_sort(arr2, 3);
    assert(sorted2 != NULL && "Same elements array should return non-NULL");
    assert(sorted2[0] == 1 && sorted2[1] == 1 && sorted2[2] == 1 && "Array with same elements should remain unchanged");
    assert(is_sorted(sorted2, 3) && "Array with same elements should be sorted");
    free(sorted2);
    printf("Test same elements passed\n");
}

void test_reverse_sorted() {
    int arr[] = {5, 4, 3, 2, 1};
    int* sorted = selection_sort(arr, 5);
    assert(sorted != NULL && "Reverse sorted array should return non-NULL");
    assert(sorted[0] == 1 && sorted[1] == 2 && sorted[2] == 3 && sorted[3] == 4 && sorted[4] == 5 && "Reverse sorted array should be sorted");
    assert(is_sorted(sorted, 5) && "Reverse sorted array should be sorted");
    free(sorted);
    printf("Test reverse sorted passed\n");
}

void test_random_arrays() {
    srand(time(NULL));
    for (int i = 0; i < 100; i++) {
        size_t size = rand() % 101;
        int* arr = (int*)malloc(size * sizeof(int));
        if (arr == NULL) continue;

        for (size_t j = 0; j < size; j++) {
            arr[j] = (rand() % 2001) - 1000;
        }

        int* sorted = selection_sort(arr, size);
        assert(sorted != NULL && "Random array should return non-NULL");
        assert(is_sorted(sorted, size) && "Random array should be sorted");

        int* arr_sorted = (int*)malloc(size * sizeof(int));
        if (arr_sorted != NULL) {
            memcpy(arr_sorted, arr, size * sizeof(int));
            qsort(arr_sorted, size, sizeof(int), (int (*)(const void*, const void*))strcmp);
            for (size_t j = 0; j < size; j++) {
                int found = 0;
                for (size_t k = 0; k < size; k++) {
                    if (sorted[j] == arr_sorted[k]) {
                        found = 1;
                        break;
                    }
                }
                assert(found && "Random array elements should be preserved");
            }
            free(arr_sorted);
        }

        free(arr);
        free(sorted);
    }
    printf("Test random arrays passed\n");
}

int main() {
    test_empty_array();
    test_single_element();
    test_two_elements();
    test_even_length();
    test_odd_length();
    test_same_elements();
    test_reverse_sorted();
    test_random_arrays();
    printf("All tests passed successfully!\n");
    return 0;
}
