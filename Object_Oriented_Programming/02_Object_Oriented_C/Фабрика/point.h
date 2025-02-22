#pragma once

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include "new.h"
#include "point.h"

struct Point { 
	const void *class;
	int x, y;
};

extern const void *Point;
