#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

// --- Квадратичные сортировки (O(n^2)) ---

void bubble_sort(int arr[], int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                int temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

void insertion_sort(int arr[], int n) {
    for (int i = 1; i < n; i++) {
        int key = arr[i], j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

void selection_sort(int arr[], int n) {
    for (int i = 0; i < n - 1; i++) {
        int min_idx = i;
        for (int j = i + 1; j < n; j++) {
            if (arr[j] < arr[min_idx])
                min_idx = j;
        }
        int temp = arr[i];
        arr[i] = arr[min_idx];
        arr[min_idx] = temp;
    }
}

// --- Квази-линейные сортировки (O(n log n)) ---

void quick_sort(int arr[], int low, int high) {
    if (low >= high) return;

    int pivot = arr[high], i = low - 1;
    for (int j = low; j < high; j++) {
        if (arr[j] < pivot) {
            i++;
            int temp = arr[i];
            arr[i] = arr[j];
            arr[j] = temp;
        }
    }
    int temp = arr[i + 1];
    arr[i + 1] = arr[high];
    arr[high] = temp;

    int pi = i + 1;
    quick_sort(arr, low, pi - 1);
    quick_sort(arr, pi + 1, high);
}

void merge(int arr[], int left, int mid, int right) {
    int n1 = mid - left + 1, n2 = right - mid;
    int L[n1], R[n2];

    for (int i = 0; i < n1; i++) L[i] = arr[left + i];
    for (int i = 0; i < n2; i++) R[i] = arr[mid + 1 + i];

    int i = 0, j = 0, k = left;
    while (i < n1 && j < n2) {
        if (L[i] <= R[j]) arr[k++] = L[i++];
        else arr[k++] = R[j++];
    }
    while (i < n1) arr[k++] = L[i++];
    while (j < n2) arr[k++] = R[j++];
}

void merge_sort(int arr[], int left, int right) {
    if (left >= right) return;
    int mid = left + (right - left) / 2;
    merge_sort(arr, left, mid);
    merge_sort(arr, mid + 1, right);
    merge(arr, left, mid, right);
}

void heapify(int arr[], int n, int i) {
    int largest = i, left = 2 * i + 1, right = 2 * i + 2;
    if (left < n && arr[left] > arr[largest]) largest = left;
    if (right < n && arr[right] > arr[largest]) largest = right;
    if (largest != i) {
        int temp = arr[i];
        arr[i] = arr[largest];
        arr[largest] = temp;
        heapify(arr, n, largest);
    }
}

void heap_sort(int arr[], int n) {
    for (int i = n / 2 - 1; i >= 0; i--) heapify(arr, n, i);
    for (int i = n - 1; i > 0; i--) {
        int temp = arr[0];
        arr[0] = arr[i];
        arr[i] = temp;
        heapify(arr, i, 0);
    }
}

// --- Функции для тестирования и измерения времени ---

void copy_array(int src[], int dest[], int n) {
    for (int i = 0; i < n; i++) dest[i] = src[i];
}

void test_sorts() {
    int test_cases[][10] = {
        {},
        {42},
        {5, 2, 8, 3, 1},
        {3, 3, 3, 3},
        {-2, -5, 0, 7, 1},
        {100, 99, 98, 97, 96}
    };
    int n_cases = SIZE(test_cases);

    void (*sorts[])(int[], int) = {bubble_sort, insertion_sort, selection_sort, heap_sort};
    char *names[] = {"Bubble", "Insertion", "Selection", "Heap"};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < n_cases; j++) {
            int arr[10];
            copy_array(test_cases[j], arr, 10);
            sorts[i](arr, 10);
            for (int k = 1; k < 10; k++) {
                if (arr[k - 1] > arr[k]) {
                    printf("Ошибка в %s\n", names[i]);
                    return;
                }
            }
        }
    }
    printf("Все тесты пройдены!\n");
}

void benchmark() {
    int sizes[] = {100, 500, 1000, 5000, 10000, 20000};
    int num_sizes = SIZE(sizes);
    void (*sorts[])(int[], int) = {bubble_sort, insertion_sort, selection_sort, heap_sort};
    char *names[] = {"Bubble", "Insertion", "Selection", "Heap"};

    printf("%-10s %-10s %-10s %-10s %-10s\n", "Размер", "Bubble", "Insert", "Select", "Heap");

    for (int i = 0; i < num_sizes; i++) {
        int size = sizes[i];
        int arr[size], copy[size];
        
        for (int j = 0; j < size; j++) arr[j] = rand() % 10000;

        printf("%-10d", size);
        for (int j = 0; j < 4; j++) {
            copy_array(arr, copy, size);
            clock_t start = clock();
            sorts[j](copy, size);
            clock_t end = clock();
            printf("%-10.4f", (double)(end - start) / CLOCKS_PER_SEC);
        }
        printf("\n");
    }
}

// --- Главная функция ---
int main() {
    srand(time(NULL));
    test_sorts();
    benchmark();
    return 0;
}
