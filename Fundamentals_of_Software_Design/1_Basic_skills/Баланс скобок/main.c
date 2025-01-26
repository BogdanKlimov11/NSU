#include <stdio.h>
#include <assert.h>

int check_brackets(const char *expression) {
    int balance = 0; // Счетчик баланса скобок

    for (int i = 0; expression[i] != '\0'; i++) {
        if (expression[i] == '(') {
            balance++; // Увеличиваем баланс для открывающей скобки
        } else if (expression[i] == ')') {
            balance--; // Уменьшаем баланс для закрывающей скобки
        }
        if (balance < 0) {
            return 0; // Если закрывающих скобок больше, чем открывающих
        }
    }

    return balance == 0; // Баланс должен быть равен нулю
}

// Тесты
int main() {
    assert(check_brackets("") == 1);
    assert(check_brackets("()") == 1);
    assert(check_brackets("((()))()(())") == 1);
    assert(check_brackets("(()())") == 1);
    assert(check_brackets(")(") == 0);
    assert(check_brackets("((())))") == 0);
    assert(check_brackets("(()") == 0);

    printf("Все тесты пройдены.\n");
    return 0;
}
