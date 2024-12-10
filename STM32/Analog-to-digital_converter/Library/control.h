#pragma once
#include "stm32f0xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "button.h"
#include "object.h"
#include "driver.h"
#include "multim.h"

void ButtonsControl(ConfButton buttons[], ScreenObject screenObject[], uint8_t countObj, uint8_t* currMovedObj);
void ChangeCountInterruptsWait(Multim* multim, ConfButton buttons[]);
