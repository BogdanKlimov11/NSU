#include <stdio.h>
#include <stdlib.h>

void guess_number() {
    int low = 1, high = 100, mid;
    char answer;

    printf("Загадайте число от 1 до 100.\n");

    while (low <= high) {
        mid = (low + high) / 2;
        printf("Загаданное число больше или равно %d? (д/н): ", mid);
        scanf(" %c", &answer);

        if (answer == 'д') {
            low = mid;
        } else if (answer == 'н') {
            high = mid - 1;
        } else {
            printf("Некорректный ввод, пожалуйста, введите 'д' или 'н'.\n");
            continue;
        }

        if (low == high) {
            printf("Загаданное число - %d\n", low);
            break;
        }
    }
}

int main() {
    char play_again;

    do {
        guess_number();
        printf("Ещё один раунд? (д/н): ");
        scanf(" %c", &play_again);
    } while (play_again == 'д');

    printf("Спасибо за игру!\n");
    return 0;
}
