#include <stdlib.h>
#include <string.h>

#include "selection_sort.h"

int* selection_sort(const int* arr, size_t size) {
    int* arr_copy = (int*)malloc(size * sizeof(int));
    if (arr_copy == NULL) return NULL;
    memcpy(arr_copy, arr, size * sizeof(int));

    for (size_t i = 0; i < size - 1; i++) {
        size_t min_idx = i;
        for (size_t j = i + 1; j < size; j++) {
            if (arr_copy[j] < arr_copy[min_idx]) {
                min_idx = j;
            }
        }
        if (min_idx != i) {
            int temp = arr_copy[i];
            arr_copy[i] = arr_copy[min_idx];
            arr_copy[min_idx] = temp;
        }
    }

    return arr_copy;
}
