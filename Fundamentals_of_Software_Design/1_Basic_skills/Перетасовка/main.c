#include <stdio.h>
#include <stdlib.h>
#include <time.h>

void shuffle_array(int arr[], size_t count) {
    // Инициализация генератора случайных чисел
    srand((unsigned)time(NULL));
    
    for (size_t i = count - 1; i > 0; i--) {
        // Генерация случайного индекса от 0 до i
        size_t j = rand() % (i + 1);
        // Обмен элементов arr[i] и arr[j]
        int temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
    }
}

int main() {
    int array[] = {1, 2, 3, 4, 5};
    size_t size = sizeof(array) / sizeof(array[0]);

    printf("Исходный массив: ");
    for (size_t i = 0; i < size; i++) {
        printf("%d ", array[i]);
    }
    printf("\n");

    shuffle_array(array, size);

    printf("Перемешанный массив: ");
    for (size_t i = 0; i < size; i++) {
        printf("%d ", array[i]);
    }
    printf("\n");

    return 0;
}
