#include <stdio.h>

// Функция для оборачивания массива
void reverse(int arr[], size_t count) {
    size_t left = 0;          // Указатель на начало массива
    size_t right = count - 1; // Указатель на конец массива

    while (left < right) {
        // Меняем местами элементы
        int temp = arr[left];
        arr[left] = arr[right];
        arr[right] = temp;

        // Сдвигаем указатели
        left++;
        right--;
    }
}

// Функция для вывода массива
void print_array(const int arr[], size_t count) {
    for (size_t i = 0; i < count; i++) {
        printf("%d ", arr[i]);
    }
    printf("\n");
}

int main() {
    int arr[] = {1, 2, 3, 4, 5};
    size_t count = sizeof(arr) / sizeof(arr[0]);

    printf("Исходный массив: ");
    print_array(arr, count);

    reverse(arr, count);

    printf("Перевернутый массив: ");
    print_array(arr, count);

    return 0;
}
