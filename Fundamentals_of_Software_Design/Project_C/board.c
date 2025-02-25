#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "board.h"

/* Initializing the playing field */
void initBoard(int size,cell board[size][size]) {
    for(int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            board[i][j].isMine = 0;
            board[i][j].picture = '*';
            board[i][j].minesNearby = 0;
        }
    }
}

/* Place mines randomly on the playing field */
void placeMines(int size,cell board[size][size], int mines) {
    srand(time(0));
    int countMins = 0;
    while (countMins < mines) {
        int i = rand() % size;
        int j = rand() % size;
        if (board[i][j].isMine == 0) {
            board[i][j].isMine = 1;
            countMins++;
        }
    }
}

/* Counts mines around one cell */
void __countMines(int size,cell board[size][size], int row, int col) {
    int count = 0;
    for (int i = row - 1; i <= row + 1; i++) {
        for (int j = col - 1; j <= col + 1; j++) {
            if (i >= 0 && i < size && j >= 0 && j < size) {
                if (board[i][j].isMine == 1) {
                    count++;
                }
            }
        }
    }
    board[row][col].minesNearby = count;
}

/* Counts mines around all cells */
void countNearbyMines(int size,cell board[size][size]) {
    for(int row = 0; row < size; row++) {
        for (int col = 0; col < size; col++) {
            __countMines(size, board, row, col);
        }
    }
}

/* Drawing the playing field */
void drawBoard(int size,cell board[size][size]) {
    printf("  |");
    for(int i = 0; i < size; i++) {
        printf("%2d|", i);
    }
    printf("\n");
    for(int i = 0; i < size; i++) {
        printf("%2d|", i);
        for (int j = 0; j < size; j++) {
            printf(" %c|", board[i][j].picture);
        }
        printf("\n");
    }
}

/* Breadth First Search */
void __BFS(int size,cell board[size][size], int row, int col) {
    if (board[row][col].picture == '*' || board[row][col].picture == 'F') {
        if (board[row][col].minesNearby == 0) {
            board[row][col].picture = ' ';
            __BFS(size, board, row-1, col);
            __BFS(size, board, row+1, col);
            __BFS(size, board, row, col-1);
            __BFS(size, board, row, col+1);
        }
        else {
            switch (board[row][col].minesNearby) {
                case 1:
                    board[row][col].picture = '1';
                    break;
                case 2:
                    board[row][col].picture = '2';
                    break;
                case 3:
                    board[row][col].picture = '3';
                    break;
                case 4:
                    board[row][col].picture = '4';
                    break;
                case 5:
                    board[row][col].picture = '5';
                    break;
                case 6:
                    board[row][col].picture = '6';
                    break;
                case 7:
                    board[row][col].picture = '7';
                    break;
                case 8:
                    board[row][col].picture = '8';
                    break;
                default:
                    board[row][col].picture = '9';
                    break;
            }
        }
    }
}

/* Try to open the cell */
int openCell(int size,cell board[size][size], int row, int col) {
    if (board[row][col].isMine == 1) {
        return 1;
    }
    else {
        __BFS(size, board, row, col);
        return 0;
    }
}

/* Check that all cells are open */
int checkWin(int size, cell board[size][size]) {
    int win = 1;
    for(int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (board[i][j].isMine != 1) {
                if (board[i][j].picture == '*' || board[i][j].picture == 'F') {
                    win = 0;
                }
            }
        }
    }
    return win;
}

/* Put/Remove the flag */
int workFlag(int size,cell board[size][size], int row, int col, int sizeArr, int arr[sizeArr][2], int countFlags) {
    if (board[row][col].picture == '*' && countFlags <= sizeArr) {
        board[row][col].picture = 'F';
        for (int i = 0; i < sizeArr; i++) {
            if (arr[i][0] == -1 && arr[i][1] == -1) {
                arr[i][0] = row;
                arr[i][1] = col;
                countFlags++;
                return countFlags;
            }
        }
    }
    else if (board[row][col].picture == 'F') {
        board[row][col].picture = '*';
        for (int i = 0; i < sizeArr; i++) {
            if (arr[i][0] == row && arr[i][1] == col) {
                arr[i][0] = -1;
                arr[i][1] = -1;
                countFlags--;
                return countFlags;
            }
        }
    }
    else {
        return countFlags;
    }
}
