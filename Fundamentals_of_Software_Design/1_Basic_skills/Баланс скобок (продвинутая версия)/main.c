#include <stdio.h>
#include <string.h>
#include <assert.h>

#define MAX_STACK_SIZE 1000

int is_open_bracket(char ch) {
    return ch == '(' || ch == '[' || ch == '{' || ch == '<';
}

int is_close_bracket(char ch) {
    return ch == ')' || ch == ']' || ch == '}' || ch == '>';
}

int matches(char open, char close) {
    return (open == '(' && close == ')') ||
           (open == '[' && close == ']') ||
           (open == '{' && close == '}') ||
           (open == '<' && close == '>');
}

int check_brackets(const char *expression) {
    char stack[MAX_STACK_SIZE];
    int top = -1; // Индекс вершины стека

    for (int i = 0; expression[i] != '\0'; i++) {
        char ch = expression[i];

        if (is_open_bracket(ch)) {
            if (top == MAX_STACK_SIZE - 1) {
                return 0; // Переполнение стека
            }
            stack[++top] = ch; // Добавляем в стек
        } else if (is_close_bracket(ch)) {
            if (top == -1) {
                return 0; // Закрывающая скобка без соответствующей открывающей
            }
            if (!matches(stack[top], ch)) {
                return 0; // Несоответствие типов скобок
            }
            top--; // Убираем вершину стека
        }
    }

    return top == -1; // Если стек пуст, скобки сбалансированы
}

// Тесты
int main() {
    assert(check_brackets("{()[()]}<<<>>>") == 1);
    assert(check_brackets("{}()<({}<>)>") == 1);
    assert(check_brackets("[(])") == 0);
    assert(check_brackets("{{[]]}}") == 0);
    assert(check_brackets("<<[{}]>>") == 1);
    assert(check_brackets("{[()]") == 0);
    assert(check_brackets("") == 1);

    printf("Все тесты пройдены.\n");
    return 0;
}
