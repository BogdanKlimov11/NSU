#pragma once
#include "stm32f0xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "object.h"

typedef struct ScreenScene {
	uint8_t currDrawedFrame;
	uint8_t currWaitFrame;
	uint8_t currModifiedFrame;
	uint8_t currDrawedColumn;
	uint16_t frame[3][8];
	uint16_t emptyFrame[8];
	uint8_t locked;
}ScreenScene;

ScreenScene InitScene(void);
void ScreenDraw(ScreenScene* screenScene);
void ScreenSet(ScreenScene* screenScene, ScreenObject screenObject[], uint8_t size);
void Swap(uint8_t* first, uint8_t* second);
