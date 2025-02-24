#include <stdio.h>
#include <stdlib.h>

void josephus_problem(int n, int k) {
    // Создаем массив с пользователями
    int *users = (int *)malloc(n * sizeof(int));
    
    // Заполняем массив номерами пользователей
    for (int i = 0; i < n; i++) {
        users[i] = i + 1;
    }

    int index = 0;  // Индекс для отсчета
    int remaining = n;  // Количество оставшихся пользователей

    // Пока не остался один пользователь
    while (remaining > 1) {
        // Находим индекс выбывающего пользователя
        index = (index + k - 1) % remaining;

        // Выводим номер выбывающего
        printf("%d\n", users[index]);

        // Сдвигаем элементы, чтобы удалить выбывшего
        for (int i = index; i < remaining - 1; i++) {
            users[i] = users[i + 1];
        }

        remaining--;  // Уменьшаем количество оставшихся
    }

    // Последний оставшийся пользователь
    printf("%d\n", users[0]);

    free(users);  // Освобождаем память
}

int main() {
    int n, k;
    
    // Вводим количество пользователей и шаг выбывания
    printf("Введите количество пользователей (N): ");
    scanf("%d", &n);
    printf("Введите шаг выбывания (K): ");
    scanf("%d", &k);
    
    josephus_problem(n, k);

    return 0;
}
