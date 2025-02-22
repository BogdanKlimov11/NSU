#pragma once

#include <stdio.h>

#include "new.h"
#include "point.h"
#include "line.h"

struct Line {
	struct Point first;
	struct Point second;
};

extern const void *Line;
