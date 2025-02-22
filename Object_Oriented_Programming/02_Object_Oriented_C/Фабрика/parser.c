#include "parser.h"

void* createShape(const char* str) {
    char name[BUFF];
    int coords[4];

    int _tmp = sscanf_s(str, "%s %d  %d  %d  %d ", name, (unsigned int)sizeof(name) - 1, coords, coords + 1, coords + 2, coords + 3);
    // point
    if ((_strcmpi(name, "point") == 0) && (_tmp == 3)) {
        return new(Point, coords[0], coords[1]);
    }
    // line
    else if ((_strcmpi(name, "line") == 0) && (_tmp == 5)) {
        return new(Line, coords[0], coords[1], coords[2], coords[3]);
    }
    // circle
    else if ((_strcmpi(name, "circle") == 0) && (_tmp == 4)) {
        return new(Circle, coords[0], coords[1], coords[2]);
    }
    // rectangle
    else if ((_strcmpi(name, "rectangle") == 0) && (_tmp == 5)) {
        return new(Rectangle, coords[0], coords[1], coords[2], coords[3]);
    }
    return NULL;
}

void* parser(const char* path) {
    FILE* file = NULL;
    void ** shapes = NULL;
    // D_array* shapes = NULL;
    fopen_s(&file, path, "rb");

    if (!file) {
        return NULL;
    }

    void* current_shape;
    void** tmp;
    // shapes = (D_array*)darray_create(sizeof(struct Class*));
    shapes = darray_create(sizeof(struct Class*));

    if (!shapes) {
        printf("Out of range! \n");
        fclose(file);
        return NULL;
    }

    char str[BUFF];
    while (fgets(str, BUFF, file)) {
        current_shape = createShape(str);
        if (current_shape) {
            tmp = (struct Class**)darray_add(shapes);
            if (!tmp) {
                printf("Error: out of range\n");
                break;
            }

            *tmp = current_shape;
        }
    }
    fclose(file);
    return shapes;
}
