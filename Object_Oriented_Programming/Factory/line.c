#include "line.h"

static void *Line_ctor(void *_self, va_list *app) {
    struct Line *self = _self;
    ((const struct Class *)Point)->ctor(&self->first, app);
    ((const struct Class *)Point)->ctor(&self->second, app);

    return self;
}

#define x(p) (((const struct Point *)(p)) -> x)
#define y(p) (((const struct Point *)(p)) -> y)

static void Line_draw(const void * _self) {
    const struct Line *self = _self;
    printf("line at (%d,%d),(%d,%d)\n", x(&self->first), y(&self->first), x(&self->second), y(&self->second));
}

static const struct Class _Line = {
    sizeof(struct Line),
    Line_ctor,
    0,
    Line_draw
};

const void *Line = &_Line;
