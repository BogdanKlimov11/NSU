#pragma once
#include "stm32f0xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "object.h"
#include "object.h"

typedef struct Multim{
	uint8_t currCalc;
	uint8_t currInterrupt;
	uint8_t countInterruptsWait;
	uint8_t bufferSize;
	uint8_t halfBufferSize;
	uint8_t flagRunCalcAverAndWrite;
}Multim;

Multim InitMultim(uint8_t buffersize, uint8_t initCountInterruptsWait);

void CheckCalc(Multim* multim);
void CalcAver(Multim* multim, uint8_t BufferADC[], uint8_t templ[8]);
void WriteAver(uint8_t aver, uint8_t templ[8]);
