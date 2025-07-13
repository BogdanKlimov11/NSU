#include "binary_search.h"

int binary_search(const int* arr, size_t size, int key) {
    if (arr == NULL || size == 0) {
        return -1;
    }

    size_t left = 0;
    size_t right = size - 1;

    while (left <= right) {
        size_t mid = left + (right - left) / 2;
        
        if (arr[mid] == key) {
            return mid;
        }
        
        if (arr[mid] < key) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }

    return -1;
}
