#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define SIZE 1000000  // Размер массива

// Сортировка вставками
void insertion_sort(int arr[], int n) {
    for (int i = 1; i < n; i++) {
        int key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j = j - 1;
        }
        arr[j + 1] = key;
    }
}

// Быстрая сортировка
int partition(int arr[], int low, int high) {
    int pivot = arr[high];
    int i = low - 1;
    for (int j = low; j < high; j++) {
        if (arr[j] <= pivot) {
            i++;
            int temp = arr[i];
            arr[i] = arr[j];
            arr[j] = temp;
        }
    }
    int temp = arr[i + 1];
    arr[i + 1] = arr[high];
    arr[high] = temp;
    return i + 1;
}

void quick_sort(int arr[], int low, int high) {
    if (low < high) {
        int pi = partition(arr, low, high);
        quick_sort(arr, low, pi - 1);
        quick_sort(arr, pi + 1, high);
    }
}

// Поразрядная сортировка (по байтам)
void counting_sort_by_byte(int arr[], int n, int byte_index) {
    int count[256] = {0};
    int output[n];

    // Подсчитываем вхождения каждого байта
    for (int i = 0; i < n; i++) {
        int byte_value = (arr[i] >> (8 * byte_index)) & 0xFF;
        count[byte_value]++;
    }

    // Модифицируем массив count так, чтобы он содержал позиции элементов в output
    for (int i = 1; i < 256; i++) {
        count[i] += count[i - 1];
    }

    // Строим отсортированный массив
    for (int i = n - 1; i >= 0; i--) {
        int byte_value = (arr[i] >> (8 * byte_index)) & 0xFF;
        output[count[byte_value] - 1] = arr[i];
        count[byte_value]--;
    }

    // Копируем отсортированные элементы обратно в оригинальный массив
    for (int i = 0; i < n; i++) {
        arr[i] = output[i];
    }
}

void radix_sort(int arr[], int n) {
    for (int byte_index = 0; byte_index < 4; byte_index++) {  // Для 32-битных чисел 4 байта
        counting_sort_by_byte(arr, n, byte_index);
    }
}

// Функция для сравнения в qsort
int compare(const void *a, const void *b) {
    return (*(int*)a - *(int*)b);
}

// Функция для генерации случайного массива
void generate_random_array(int arr[], int n) {
    for (int i = 0; i < n; i++) {
        arr[i] = rand() % 1000000;
    }
}

int main() {
    srand(time(NULL));

    int arr1[SIZE], arr2[SIZE], arr3[SIZE], arr4[SIZE];
    
    // Генерируем случайные данные для тестов
    generate_random_array(arr1, SIZE);
    for (int i = 0; i < SIZE; i++) arr2[i] = arr1[i];
    for (int i = 0; i < SIZE; i++) arr3[i] = arr1[i];
    for (int i = 0; i < SIZE; i++) arr4[i] = arr1[i];

    clock_t start, end;

    // Тестируем сортировку вставками
    start = clock();
    insertion_sort(arr1, SIZE);
    end = clock();
    printf("Сортировка вставками: %.2f секунд\n", (double)(end - start) / CLOCKS_PER_SEC);

    // Тестируем быструю сортировку
    start = clock();
    quick_sort(arr2, 0, SIZE - 1);
    end = clock();
    printf("Быстрая сортировка: %.2f секунд\n", (double)(end - start) / CLOCKS_PER_SEC);

    // Тестируем поразрядную сортировку
    start = clock();
    radix_sort(arr3, SIZE);
    end = clock();
    printf("Поразрядная сортировка: %.2f секунд\n", (double)(end - start) / CLOCKS_PER_SEC);

    // Тестируем библиотечную qsort
    start = clock();
    qsort(arr4, SIZE, sizeof(int), compare);
    end = clock();
    printf("Библиотечная qsort: %.2f секунд\n", (double)(end - start) / CLOCKS_PER_SEC);

    return 0;
}
