#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>

void selection_sort(int arr[], int n) {
    for (int i = 0; i < n - 1; i++) {
        int min_idx = i;
        for (int j = i + 1; j < n; j++) {
            if (arr[j] < arr[min_idx]) {
                min_idx = j;
            }
        }
        int temp = arr[i];
        arr[i] = arr[min_idx];
        arr[min_idx] = temp;
    }
}

void test() {
    int arr1[] = {};
    selection_sort(arr1, 0);

    int arr2[] = {5};
    selection_sort(arr2, 1);
    assert(arr2[0] == 5);

    int arr3[] = {3, 1, 2};
    selection_sort(arr3, 3);
    assert(arr3[0] == 1 && arr3[1] == 2 && arr3[2] == 3);

    int arr4[] = {5, 4, 3, 2, 1};
    selection_sort(arr4, 5);
    assert(arr4[0] == 1 && arr4[1] == 2 && arr4[2] == 3 && arr4[3] == 4 && arr4[4] == 5);

    // Тест случайных массивов
    srand(time(NULL));
    for (int t = 0; t < 10; t++) {
        int n = rand() % 20;
        int arr[n], sorted[n];
        for (int i = 0; i < n; i++) {
            arr[i] = sorted[i] = rand() % 201 - 100;
        }

        selection_sort(arr, n);

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
