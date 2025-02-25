#include <stdio.h>
#include <stdlib.h>

void swap(int* a, int* b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

void generate_permutations(int* arr, int left, int right) {
    if (left == right) {
        for (int i = 0; i <= right; i++) {
            printf("%d ", arr[i]);
        }
        printf("\n");
        return;
    }

    for (int i = left; i <= right; i++) {
        swap(&arr[left], &arr[i]);
        generate_permutations(arr, left + 1, right);
        swap(&arr[left], &arr[i]);  // Возвращаем массив в исходное состояние
    }
}

void permutations(int n) {
    if (n <= 0) return;

    int* arr = malloc(n * sizeof(int));
    if (!arr) {
        perror("Ошибка выделения памяти");
        return;
    }

    for (int i = 0; i < n; i++) {
        arr[i] = i;
    }

    generate_permutations(arr, 0, n - 1);
    free(arr);
}

int main() {
    int n = 3;  // Измените на нужное значение
    permutations(n);
    return 0;
}
