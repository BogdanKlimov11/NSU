#include "rectangle.h"

static void *Rectangle_ctor(void *_self, va_list *app) {
    struct Rectangle *self = _self;
    ((const struct Class *)Point)->ctor(&self->first, app);
    ((const struct Class *)Point)->ctor(&self->second, app);

    return self;
}

#define x(p) (((const struct Point *)(p)) -> x)
#define y(p) (((const struct Point *)(p)) -> y)

static void Rectangle_draw(const void * _self) {
    const struct Rectangle *self = _self;
    printf("rectangle at (%d,%d),(%d,%d)\n", x(&self->first), y(&self->first), x(&self->second), y(&self->second));
}

static const struct Class _Rectangle = {
    sizeof(struct Rectangle),
    Rectangle_ctor,
    0,
    Rectangle_draw
};

const void *Rectangle = &_Rectangle;
