#include <stdio.h>
#include <stdlib.h>

void swap(int* a, int* b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

void heapify(int* arr, int n, int i) {
    int largest = i; // Инициализация корня как самого большого
    int left = 2 * i + 1; // Левый дочерний узел
    int right = 2 * i + 2; // Правый дочерний узел

    // Если левый дочерний элемент больше корня
    if (left < n && arr[left] > arr[largest]) {
        largest = left;
    }

    // Если правый дочерний элемент больше самого большого
    if (right < n && arr[right] > arr[largest]) {
        largest = right;
    }

    // Если самый большой элемент не корень
    if (largest != i) {
        swap(&arr[i], &arr[largest]);

        // Рекурсивно делаем кучу
        heapify(arr, n, largest);
    }
}

void heap_sort(int* arr, int n) {
    // Строим кучу (max-heap)
    for (int i = n / 2 - 1; i >= 0; i--) {
        heapify(arr, n, i);
    }

    // Извлекаем элементы из кучи по одному
    for (int i = n - 1; i > 0; i--) {
        swap(&arr[0], &arr[i]); // Перемещаем текущий корень в конец
        heapify(arr, i, 0); // Восстанавливаем кучу
    }
}

void print_array(int* arr, int n) {
    for (int i = 0; i < n; i++) {
        printf("%d ", arr[i]);
    }
    printf("\n");
}

int main() {
    int arr1[] = {10, 20, 30, 40, 50};
    int n1 = sizeof(arr1) / sizeof(arr1[0]);
    heap_sort(arr1, n1);
    print_array(arr1, n1); // Вывод: 10 20 30 40 50

    int arr2[] = {5};
    int n2 = sizeof(arr2) / sizeof(arr2[0]);
    heap_sort(arr2, n2);
    print_array(arr2, n2); // Вывод: 5

    int arr3[] = {3, 1, 4, 1, 5, 9, 2, 6};
    int n3 = sizeof(arr3) / sizeof(arr3[0]);
    heap_sort(arr3, n3);
    print_array(arr3, n3); // Вывод: 1 1 2 3 4 5 6 9

    return 0;
}
