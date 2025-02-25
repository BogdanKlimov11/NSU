#include <stdio.h>
#include <assert.h>

int binary_search(int arr[], int size, int key) {
    int left = 0, right = size - 1;
    while (left <= right) {
        int mid = left + (right - left) / 2;
        if (arr[mid] == key)
            return mid;
        else if (arr[mid] < key)
            left = mid + 1;
        else
            right = mid - 1;
    }
    return -1;  // Элемент не найден
}

void test() {
    int arr1[] = {};
    assert(binary_search(arr1, 0, 10) == -1);  // Пустой массив

    int arr2[] = {5};
    assert(binary_search(arr2, 1, 5) == 0);  // Один элемент (есть)
    assert(binary_search(arr2, 1, 10) == -1);  // Один элемент (нет)

    int arr3[] = {1, 3, 5, 7, 9};
    assert(binary_search(arr3, 5, 1) == 0);  // Первый элемент
    assert(binary_search(arr3, 5, 9) == 4);  // Последний элемент
    assert(binary_search(arr3, 5, 5) == 2);  // Средний элемент
    assert(binary_search(arr3, 5, 6) == -1);  // Элемента нет

    int arr4[] = {2, 4, 6, 8, 10, 12};
    assert(binary_search(arr4, 6, 6) == 2);  // Четный размер
    assert(binary_search(arr4, 6, 12) == 5);  // Последний элемент
    assert(binary_search(arr4, 6, 3) == -1);  // Элемента нет

    printf("Все тесты пройдены!\n");
}

int main() {
    test();
    return 0;
}
