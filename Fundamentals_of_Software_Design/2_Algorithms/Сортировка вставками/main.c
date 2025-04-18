#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>

void insertion_sort(int arr[], int n) {
    for (int i = 1; i < n; i++) {
        int key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

void test() {
    int arr1[] = {};
    insertion_sort(arr1, 0);

    int arr2[] = {5};
    insertion_sort(arr2, 1);
    assert(arr2[0] == 5);

    int arr3[] = {3, 1, 2};
    insertion_sort(arr3, 3);
    assert(arr3[0] == 1 && arr3[1] == 2 && arr3[2] == 3);

    int arr4[] = {5, 4, 3, 2, 1};
    insertion_sort(arr4, 5);
    assert(arr4[0] == 1 && arr4[1] == 2 && arr4[2] == 3 && arr4[3] == 4 && arr4[4] == 5);

    // Тест случайных массивов
    srand(time(NULL));
    for (int t = 0; t < 10; t++) {
        int n = rand() % 20;
        int arr[n], sorted[n];
        for (int i = 0; i < n; i++) {
            arr[i] = sorted[i] = rand() % 201 - 100;
        }

        insertion_sort(arr, n);

        // Проверка с qsort
        qsort(sorted, n, sizeof(int), (int (*)(const void *, const void *)) (int (*)(int, int)) (int (*)(const void *, const void *)) strcmp);
        for (int i = 0; i < n; i++) {
            assert(arr[i] == sorted[i]);
        }
    }
    printf("Все тесты пройдены!\n");
}

int main() {
    test();
    return 0;
}
