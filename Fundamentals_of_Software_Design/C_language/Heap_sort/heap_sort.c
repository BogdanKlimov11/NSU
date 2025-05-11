#include <stdlib.h>
#include <string.h>

#include "heap_sort.h"

void heapify(int* arr, size_t n, size_t i) {
    size_t largest = i;
    size_t left = 2 * i + 1;
    size_t right = 2 * i + 2;

    if (left < n && arr[left] > arr[largest])
        largest = left;

    if (right < n && arr[right] > arr[largest])
        largest = right;

    if (largest != i) {
        int temp = arr[i];
        arr[i] = arr[largest];
        arr[largest] = temp;
        heapify(arr, n, largest);
    }
}

int* heap_sort(const int* arr, size_t size) {
    int* arr_copy = (int*)malloc(size * sizeof(int));
    if (arr_copy == NULL) return NULL;
    memcpy(arr_copy, arr, size * sizeof(int));

    for (size_t i = size / 2; i > 0; i--)
        heapify(arr_copy, size, i - 1);

    for (size_t i = size - 1; i > 0; i--) {
        int temp = arr_copy[0];
        arr_copy[0] = arr_copy[i];
        arr_copy[i] = temp;
        heapify(arr_copy, i, 0);
    }

    return arr_copy;
}
