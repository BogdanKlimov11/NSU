#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// --- Сортировка выбором ---
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

// --- Сортировка вставками ---
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

// --- Однонаправленная пузырьковая сортировка ---
void bubble_sort(int arr[], int n) {
    for (int i = 0; i < n - 1; i++) {
        int swapped = 0;
        for (int j = 0; j < n - 1 - i; j++) {
            if (arr[j] > arr[j + 1]) {
                int temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
                swapped = 1;
            }
        }
        if (!swapped) break;
    }
}

// --- Двунаправленная (cocktail) пузырьковая сортировка ---
void cocktail_sort(int arr[], int n) {
    int swapped = 1, start = 0, end = n - 1;
    while (swapped) {
        swapped = 0;
        for (int i = start; i < end; i++) {
            if (arr[i] > arr[i + 1]) {
                int temp = arr[i];
                arr[i] = arr[i + 1];
                arr[i + 1] = temp;
                swapped = 1;
            }
        }
        end--;
        if (!swapped) break;
        swapped = 0;
        for (int i = end; i > start; i--) {
            if (arr[i] < arr[i - 1]) {
                int temp = arr[i];
                arr[i] = arr[i - 1];
                arr[i - 1] = temp;
                swapped = 1;
            }
        }
        start++;
    }
}

// --- Функция тестирования ---
void test_sorts() {
    int test_cases[][7] = {
        {},                     // Пустой массив
        {1},                    // Один элемент
        {3, 2, 1},              // Обратный порядок
        {1, 2, 3},              // Уже отсортирован
        {5, 3, 8, 4, 2, 7, 1},  // Случайный порядок
        {1, 1, 1, 1, 1, 1, 1}   // Все элементы одинаковые
    };
    int sizes[] = {0, 1, 3, 3, 7, 7};
    
    void (*sorts[])(int*, int) = {selection_sort, insertion_sort, bubble_sort, cocktail_sort};
    char *sort_names[] = {"Selection Sort", "Insertion Sort", "Bubble Sort", "Cocktail Sort"};
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 6; j++) {
            int arr[7];
            for (int k = 0; k < sizes[j]; k++) arr[k] = test_cases[j][k];

            sorts[i](arr, sizes[j]);

            for (int k = 1; k < sizes[j]; k++) {
                if (arr[k - 1] > arr[k]) {
                    printf("❌ %s failed on test case %d\n", sort_names[i], j);
                    return;
                }
            }
        }
    }
    printf("Все тесты пройдены!\n");
}

// --- Сравнение времени работы ---
void benchmark() {
    int sizes[] = {100, 500, 1000, 2000, 3000};
    void (*sorts[])(int*, int) = {selection_sort, insertion_sort, bubble_sort, cocktail_sort};
    char *sort_names[] = {"Selection Sort", "Insertion Sort", "Bubble Sort", "Cocktail Sort"};

    for (int i = 0; i < 5; i++) {
        int size = sizes[i];
        printf("\nРазмер массива: %d\n", size);

        int *arr = malloc(size * sizeof(int));
        int *copy = malloc(size * sizeof(int));

        for (int j = 0; j < 4; j++) {
            srand(time(NULL));
            for (int k = 0; k < size; k++) arr[k] = rand() % (size * 10);
            
            for (int k = 0; k < size; k++) copy[k] = arr[k];

            clock_t start = clock();
            sorts[j](copy, size);
            clock_t end = clock();

            printf("%s: %.5f сек\n", sort_names[j], (double)(end - start) / CLOCKS_PER_SEC);
        }
        
        free(arr);
        free(copy);
    }
}

// --- Запуск программы ---
int main() {
    test_sorts();
    benchmark();
    return 0;
}
