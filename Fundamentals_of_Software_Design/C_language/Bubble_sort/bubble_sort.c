#include <stdlib.h>
#include <string.h>

#include "bubble_sort.h"

int* bubble_sort(const int* arr, size_t size) {
    int* arr_copy = (int*)malloc(size * sizeof(int));
    if (arr_copy == NULL) return NULL;
    memcpy(arr_copy, arr, size * sizeof(int));

    for (size_t i = 0; i < size; i++) {
        for (size_t j = 0; j < size - i - 1; j++) {
            if (arr_copy[j] > arr_copy[j + 1]) {
                int temp = arr_copy[j];
                arr_copy[j] = arr_copy[j + 1];
                arr_copy[j + 1] = temp;
            }
        }
    }

    return arr_copy;
}
