#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "quick_sort.h"

void swap(int* a, int* b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

size_t partition(int* arr, size_t low, size_t high) {
    size_t pivot_idx = low + rand() % (high - low + 1);
    swap(&arr[pivot_idx], &arr[high]);
    
    int pivot = arr[high];
    size_t i = low;
    
    for (size_t j = low; j < high; j++) {
        if (arr[j] <= pivot) {
            swap(&arr[i], &arr[j]);
            i++;
        }
    }
    swap(&arr[i], &arr[high]);
    return i;
}

void quick_sort_helper(int* arr, size_t low, size_t high) {
    if (low < high) {
        size_t pi = partition(arr, low, high);
        if (pi > 0) quick_sort_helper(arr, low, pi - 1);
        quick_sort_helper(arr, pi + 1, high);
    }
}

int* quick_sort(const int* arr, size_t size) {
    int* arr_copy = (int*)malloc(size * sizeof(int));
    if (arr_copy == NULL) return NULL;
    memcpy(arr_copy, arr, size * sizeof(int));

    srand(time(NULL));
    
    if (size > 1) {
        quick_sort_helper(arr_copy, 0, size - 1);
    }
    
    return arr_copy;
}
