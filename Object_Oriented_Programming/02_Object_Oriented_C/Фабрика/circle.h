#pragma once

#include <stdio.h>
#include "new.h"
#include "point.h"
#include "circle.h"

struct Circle {
    const struct Point _;
    int radius;
};

extern const void *Circle;
