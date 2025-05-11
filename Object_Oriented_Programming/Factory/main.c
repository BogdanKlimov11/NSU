#include <stdio.h>
#include <stdlib.h>

#include "new.h"
#include "point.h"
#include "parser.h"
#include "darray.h"
#include "draw.h"

int main(int argc, char **argv) {
    char filename[100] = { 0 };
    printf("Enter file name:\n");
    gets_s(filename, sizeof(filename) - 1);
    void **shapes = parser(filename);
    //D_array* shapes = parser(filename);
    draw_all(shapes);
    darray_destroy(shapes, delete);
    return 0;
}
