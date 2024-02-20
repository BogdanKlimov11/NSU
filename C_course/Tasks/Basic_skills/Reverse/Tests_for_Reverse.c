#include <assert.h>

#include "Reverse.h"

/* Тесты для оборачивания массива */

int main() {

    // Случай size нечетное:
    int arr1[] = {1, 2, 3, 4, 5};
    int expected1[] = {5, 4, 3, 2, 1};
    size_t count1 = sizeof(arr1) / sizeof(arr1[0]);
    reverse(arr1, count1);
    for (size_t i = 0; i < count1; i++)
    {
        assert(arr1[i] == expected1[i]);
    }

    // Случай size четное:
    int arr2[] = {9, 8, 7, 6};
    int expected2[] = {6, 7, 8, 9};
    size_t count2 = sizeof(arr2) / sizeof(arr2[0]);
    reverse(arr2, count2);
    for (size_t i = 0; i < count2; i++)
    {
        assert(arr2[i] == expected2[i]);
    }

    return 0;
}