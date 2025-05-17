#include <stdlib.h>
#include <string.h>

#include "insertion_sort.h"

int* insertion_sort(const int* arr, size_t size) {
    int* arr_copy = (int*)malloc(size * sizeof(int));
    if (arr_copy == NULL) return NULL;
    memcpy(arr_copy, arr, size * sizeof(int));

    for (size_t i = 1; i < size; i++) {
        int key = arr_copy[i];
        size_t j = i;
        
        while (j > 0 && arr_copy[j - 1] > key) {
            arr_copy[j] = arr_copy[j - 1];
            j--;
        }
        arr_copy[j] = key;
    }

    return arr_copy;
}
