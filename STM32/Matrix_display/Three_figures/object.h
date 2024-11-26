#pragma once
#include "stm32f0xx.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct ScreenObject{
    uint8_t templ[8];
    int8_t posY;
    int8_t posX;
}ScreenObject;
