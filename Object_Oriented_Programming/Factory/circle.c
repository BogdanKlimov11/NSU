#include "circle.h"

static void *Circle_ctor(void *_self, va_list *app) {
    struct Circle *self = ((const struct Class *)Point)->ctor(_self, app);
    self->radius = va_arg(*app, int);
    return self;
}

#define x(p) (((const struct Point *)(p))->x)
#define y(p) (((const struct Point *)(p))->y)

static void Circle_draw(const void *_self) {
    const struct Circle *self = _self;
    printf("circle at (%d, %d) radius = %d\n", x(self), y(self), self->radius);
}

static const struct Class _Circle = {
    sizeof(struct Circle),
    Circle_ctor,
    0,
    Circle_draw
};

const void *Circle = &_Circle;
