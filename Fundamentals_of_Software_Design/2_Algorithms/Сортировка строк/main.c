#include <stdio.h>
#include <string.h>

#define MAX_CHAR 256

// Функция сортировки подсчётом по символам в строках
void counting_sort(char* arr[], int n, int char_index) {
    int count[MAX_CHAR] = {0};
    char* output[n];

    // Подсчёт количества каждого символа в текущем разряде
    for (int i = 0; i < n; i++) {
        int char_code = arr[i][char_index] ? arr[i][char_index] : 0;
        count[char_code]++;
    }

    // Преобразование count в массив позиций
    for (int i = 1; i < MAX_CHAR; i++) {
        count[i] += count[i - 1];
    }

    // Строим отсортированный массив
    for (int i = n - 1; i >= 0; i--) {
        int char_code = arr[i][char_index] ? arr[i][char_index] : 0;
        output[count[char_code] - 1] = arr[i];
        count[char_code]--;
    }

    // Копирование отсортированных строк в исходный массив
    for (int i = 0; i < n; i++) {
        arr[i] = output[i];
    }
}

// Основная функция для сортировки строк
void radix_sort_strings(char* arr[], int n) {
    // Находим максимальную длину строки
    int max_len = 0;
    for (int i = 0; i < n; i++) {
        int len = strlen(arr[i]);
        if (len > max_len) {
            max_len = len;
        }
    }

    // Сортируем строки по каждому символу (начиная с последнего)
    for (int char_index = max_len - 1; char_index >= 0; char_index--) {
        counting_sort(arr, n, char_index);
    }
}

// Функция для вывода массива строк
void print_strings(char* arr[], int n) {
    for (int i = 0; i < n; i++) {
        printf("%s ", arr[i]);
    }
    printf("\n");
}

int main() {
    char* arr[] = {"apple", "banana", "grape", "cherry", "date"};
    int n = sizeof(arr) / sizeof(arr[0]);

    printf("До сортировки:\n");
    print_strings(arr, n);

    radix_sort_strings(arr, n);

    printf("После сортировки:\n");
    print_strings(arr, n);

    return 0;
}
