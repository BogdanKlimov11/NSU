#include <stdio.h>
#include <stdlib.h>

#include "Board.h"
#include "Menu.h"

int main(void)
{
    /* Entering game parameters */
    int size;
    size = inputSizeField();
    int mines;
    mines = inputNumberMins(size);

    /* Preparing the playing field */
    cell board[size][size];
    initBoard(size, board);
    placeMines(size, board, mines);
    countNearbyMines(size, board);

    /* Creating a dynamic array for flags */
    int **arrayFlags = (int **)malloc(mines*sizeof(int *));
    for(int i = 0; i < mines; i++) {
        arrayFlags[i] = (int *)malloc(2*sizeof(int));
    }
    for (int i = 0; i < mines; i++) {
        arrayFlags[i][0] = -1;
        arrayFlags[i][1] = -1;
    }

    char action;
    int countFlags = 0;

    while (1) {
        drawBoard(size, board);
        printMenu();
        action = inputAction();
        if (action == 'X') {
            break;
        }
        else {
            int row;
            row = inputRow(size);
            int col;
            col = inputCol(size);
            if (action == 'F') {
                countFlags = workFlag(size, board, row, col, mines, arrayFlags, countFlags);
                if (countFlags == mines) {
                    int win = 1;
                    for (int i = 0; i < mines; i++) {
                        if (board[arrayFlags[i][0]][arrayFlags[i][1]].isMine != 1) {
                            win = 0;
                        }
                    }
                    if (win == 1) {
                        drawBoard(size, board);
                        printf("You won!!!\n");
                        int total;
                        while (total != 1) {
                            printf("Press 1 to exit:\n");
                            total = scanCheckInt();
                        }
                        break;
                    }
                }
            }
            else if (action == 'R') {
                if (openCell(size, board, row, col) == 1) {
                    drawBoard(size, board);
                    printf("You lose!!!\n");
                    int total;
                    while (total != 1) {
                        printf("Press 1 to exit:\n");
                        total = scanCheckInt();
                    }
                    break;
                }
                else {
                    if (checkWin(size, board) == 1) {
                        drawBoard(size, board);
                        printf("You won!!!\n");
                        int total;
                        while (total != 1) {
                            printf("Press 1 to exit:\n");
                            total = scanCheckInt();
                        }
                        break;
                    }
                }
            }
        }
    }

    /* Clearing memory for the array of flags */
    for(int i = 0; i < mines; i++) {
        free(arrayFlags[i]);
    }
    free(arrayFlags);

    return 0;
}
