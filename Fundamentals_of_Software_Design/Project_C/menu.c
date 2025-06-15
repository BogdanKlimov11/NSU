#include <stdio.h>

/* Input validation int */
int scanCheckInt() {
    int i;
    for (;;) {
        if (scanf("%d", &i) != 1) {
            printf("It's not integer, enter again:\n");
            /* Clearing the input buffer */
            scanf("%*[^\n]"); // Clearing the input buffer
        }
        else {
            while (getchar() != '\n'); // Clearing the input buffer
            break;
        }
    }
    return i;
}

/* Entering the correct field sizes */
int inputSizeField() {
    for (;;) {
        int size;
        printf("Enter field dimensions (any integer in the range [5;20]):\n");
        size = scanCheckInt();
        if (size < 21 && size > 4) {
            return size;
        }
    }
}

/* Entering the correct number of mins */
int inputNumberMins(int size) {
    for (;;) {
        int max = size * size / 4;
        int min = size * size / 10;
        printf("Enter number of mines (any integer in the range [%d;%d]):\n", min, max);
        int current;
        current = scanCheckInt();
        if (current <= max && current >= min) {
            return current;
        }
    }
}

/* Entering the correct row */
int inputRow(int size) {
    for (;;) {
        printf("Enter a string (any integer in the range [0;%d]):\n", size - 1);
        int current;
        current = scanCheckInt();
        if (current < size && current >= 0) {
            return current;
        }
    }
}

/* Entering the correct col */
int inputCol(int size) {
    for (;;) {
        printf("Enter a series (any integer in the range [0;%d]):\n", size - 1);
        int current;
        current = scanCheckInt();
        if (current < size && current >= 0) {
            return current;
        }
    }
}

/* Entering the correct action */
char inputAction() {
    char action;
    while (1) {
        printf("Enter action:\n");
        if (scanf("%c", &action) != 1) {
            printf("Error: Enter exactly one character.");
            while (getchar() != '\n'); // Clearing the input buffer
            continue;
        }
        if (action == '\n') {
            printf("You have not entered a character. Try again.\n");
            continue;
        }
        while (getchar() != '\n'); // Clearing the input buffer
        if (action == 'R' || action == 'F' || action == 'X') {
            return action;
        }
        else {
            printf("You entered an invalid character. Try again.\n");
        }
    }

}

/* Display menu */
void printMenu() {
    printf("Menu:\n");
    printf("Press R to open a cell.\n");
    printf("Press F to add/remove flag.\n");
    printf("Press X to exit.\n");
}
