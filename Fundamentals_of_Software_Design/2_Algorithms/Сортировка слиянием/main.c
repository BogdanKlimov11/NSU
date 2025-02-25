#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>

// Функция слияния двух подмассивов
void merge(int *arr, int *temp, int left, int mid, int right) {
    int i = left, j = mid, k = left;
    while (i < mid && j < right) {
        if (arr[i] < arr[j]) {
            temp[k++] = arr[i++];
        } else {
            temp[k++] = arr[j++];
        }
    }
    while (i < mid) temp[k++] = arr[i++];
    while (j < right) temp[k++] = arr[j++];
    
    for (i = left; i < right; i++) {
        arr[i] = temp[i];
    }
}

// Рекурсивная сортировка слиянием
void merge_sort_recursive(int *arr, int *temp, int left, int right) {
    if (right - left <= 1) return;
    int mid = (left + right) / 2;
    merge_sort_recursive(arr, temp, left, mid);
    merge_sort_recursive(arr, temp, mid, right);
    merge(arr, temp, left, mid, right);
}

// Основная функция сортировки
void merge_sort(int *arr, int size) {
    if (size <= 1) return;
    int *temp = (int *)malloc(size * sizeof(int));
    if (!temp) {
        fprintf(stderr, "Ошибка выделения памяти\n");
        exit(EXIT_FAILURE);
    }
    merge_sort_recursive(arr, temp, 0, size);
    free(temp);
}

// Функция для вывода массива
void print_array(int *arr, int size) {
    for (int i = 0; i < size; i++) {
        printf("%d ", arr[i]);
    }
    printf("\n");
}

// Функция для тестирования
void test() {
    int arr1[] = {};
    merge_sort(arr1, 0);
    assert(1);  // len=0, просто не падаем

    int arr2[] = {42};
    merge_sort(arr2, 1);
    assert(arr2[0] == 42); // len=1, массив не меняется

    int arr3[] = {5, 2, 8, 3, 1};
    merge_sort(arr3, 5);
    int expected3[] = {1, 2, 3, 5, 8};
    for (int i = 0; i < 5; i++) assert(arr3[i] == expected3[i]);

    int arr4[] = {3, 3, 3, 3};
    merge_sort(arr4, 4);
    assert(arr4[0] == 3 && arr4[1] == 3 && arr4[2] == 3 && arr4[3] == 3);

    printf("Все тесты пройдены!\n");
}

// Генерация случайных массивов и тест
void random_test(int num_tests, int max_size, int max_value) {
    srand(time(NULL));
    for (int t = 0; t < num_tests; t++) {
        int size = rand() % (max_size + 1);
        int *arr = (int *)malloc(size * sizeof(int));
        int *copy = (int *)malloc(size * sizeof(int));

        for (int i = 0; i < size; i++) {
            arr[i] = rand() % (2 * max_value + 1) - max_value;
            copy[i] = arr[i];
        }

        merge_sort(arr, size);
        qsort(copy, size, sizeof(int), (int (*)(const void *, const void *))compare);

        for (int i = 0; i < size; i++) assert(arr[i] == copy[i]);

        free(arr);
        free(copy);
    }
    printf("Все случайные тесты пройдены!\n");
}

// Сравнительная функция для qsort
int compare(const int *a, const int *b) {
    return (*a > *b) - (*a < *b);
}

int main() {
    test();
    random_test(100, 1000, 1000);
    return 0;
}
