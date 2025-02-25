#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>

// Функция для обмена элементов
void swap(int* a, int* b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

// Функция разбиения массива с рандомизированным выбором опорного элемента
size_t partition(int* arr, size_t low, size_t high) {
    // Выбираем случайный опорный элемент и меняем его с последним элементом
    size_t pivot_index = low + rand() % (high - low + 1);
    swap(&arr[pivot_index], &arr[high]);
    int pivot = arr[high];

    size_t i = low;
    for (size_t j = low; j < high; j++) {
        if (arr[j] < pivot) {
            swap(&arr[i], &arr[j]);
            i++;
        }
    }
    swap(&arr[i], &arr[high]);
    return i;
}

// Рекурсивная функция быстрой сортировки
void quick_sort_recursive(int* arr, size_t low, size_t high) {
    if (low < high) {
        size_t pi = partition(arr, low, high);
        if (pi > 0) quick_sort_recursive(arr, low, pi - 1);
        quick_sort_recursive(arr, pi + 1, high);
    }
}

// Основная функция быстрой сортировки
void quick_sort(int* arr, size_t len) {
    if (len > 1) {
        quick_sort_recursive(arr, 0, len - 1);
    }
}

// Функция для проверки сортировки
void test() {
    srand((unsigned)time(NULL));

    // Тесты краевых случаев
    int arr1[] = {};
    quick_sort(arr1, 0);
    assert(sizeof(arr1) / sizeof(arr1[0]) == 0);

    int arr2[] = {5};
    quick_sort(arr2, 1);
    assert(arr2[0] == 5);

    int arr3[] = {3, 1, 4, 1, 5, 9, 2, 6};
    int sorted_arr3[] = {1, 1, 2, 3, 4, 5, 6, 9};
    quick_sort(arr3, 8);
    for (size_t i = 0; i < 8; i++)
        assert(arr3[i] == sorted_arr3[i]);

    int arr4[] = {10, 20, 30, 40, 50};
    int sorted_arr4[] = {10, 20, 30, 40, 50};
    quick_sort(arr4, 5);
    for (size_t i = 0; i < 5; i++)
        assert(arr4[i] == sorted_arr4[i]);

    int arr5[] = {5, 5, 5, 5, 5};
    int sorted_arr5[] = {5, 5, 5, 5, 5};
    quick_sort(arr5, 5);
    for (size_t i = 0; i < 5; i++)
        assert(arr5[i] == sorted_arr5[i]);

    // Тест случайных массивов
    for (int t = 0; t < 10; t++) {
        size_t n = rand() % 21;  // случайное число элементов от 0 до 20
        int arr[n], sorted_arr[n];
        for (size_t i = 0; i < n; i++) {
            arr[i] = sorted_arr[i] = rand() % 101;  // числа от 0 до 100
        }
        quick_sort(arr, n);
        int compare(const void* a, const void* b) {
            return (*(int*)a - *(int*)b);
        }
        qsort(sorted_arr, n, sizeof(int), compare);
        for (size_t i = 0; i < n; i++) {
            assert(arr[i] == sorted_arr[i]);
        }
    }

    printf("Все тесты пройдены!\n");
}

int main() {
    test();
    return 0;
}
