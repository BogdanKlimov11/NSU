#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#include "new.h"
#include "point.h"
#include "circle.h"
#include "rectangle.h"
#include "line.h"
#include "darray.h"

#define BUFF 200

void* parser(const char* path);
