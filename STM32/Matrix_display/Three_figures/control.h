#pragma once
#include "stm32f0xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "button.h"
#include "object.h"

void ObjectMove(ConfButton buttons[], ScreenObject screenObject[], uint8_t countBtn, uint8_t countObj, uint8_t* currMovedObj);
