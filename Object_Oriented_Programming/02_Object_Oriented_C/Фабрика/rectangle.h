#pragma once

#include <stdio.h>

#include "point.h"
#include "new.h"

struct Rectangle {
	struct Point first;
	struct Point second;
};

extern const void *Rectangle;
