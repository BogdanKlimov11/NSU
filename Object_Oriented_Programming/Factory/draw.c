#include "draw.h"

void draw_all(void* shapes) {
    if (!shapes) {
        return;
    }

    size_t size = darray_count(shapes);
    void** tmp;
    for (size_t i = 0; i < size; i++) {
        tmp = (void**)(darray_item(shapes, i));
        if (tmp && *tmp) {
            draw(*tmp);
        }
    }
}
