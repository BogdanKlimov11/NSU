#ifndef MENU_H
#define MENU_H

/* Input validation int */
int scanCheckInt();

/* Entering the correct field sizes */
int inputSizeField();

/* Entering the correct number of mins */
int inputNumberMins(int size);

/* Entering the correct row */
int inputRow(int size);

/* Entering the correct col */
int inputCol(int size);

/* Entering the correct action */
char inputAction();

/* Display menu */
void printMenu();

#endif
