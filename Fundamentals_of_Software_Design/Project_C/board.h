#ifndef BOARD_H
#define BOARD_H

/* Single cell */
typedef struct {
    int isMine;
    char picture;
    int minesNearby;
} cell;

/* Initializing the playing field */
void initBoard(int size,cell board[size][size]);

/* Place mines randomly on the playing field */
void placeMines(int size,cell board[size][size], int mines);

/* Counts mines around all cells */
void countNearbyMines(int size,cell board[size][size]);

/* Drawing the playing field */
void drawBoard(int size,cell board[size][size]);

/* Try to open the cell */
int openCell(int size,cell board[size][size], int row, int col);

/* Put/Remove the flag */
int workFlag(int size,cell board[size][size], int row, int col, int sizeArr, int arr[sizeArr][2], int countFlags);

/* Check that all cells are open */
int checkWin(int size, cell board[size][size]);

#endif
