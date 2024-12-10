#include "driver.h"

ScreenScene InitScene(void){
	ScreenScene screenScene = 
	{
		2, 1, 0, 0, 
		{
			{0x0080, 0x0040, 0x0020, 0x0010, 0x0008, 0x0004, 0x0002, 0x0001},
			{0x0080, 0x0040, 0x0020, 0x0010, 0x0008, 0x0004, 0x0002, 0x0001},
			{0x0080, 0x0040, 0x0020, 0x0010, 0x0008, 0x0004, 0x0002, 0x0001}
		},
		{0x0080, 0x0040, 0x0020, 0x0010, 0x0008, 0x0004, 0x0002, 0x0001},
		0
	};
	return screenScene;
}

void ScreenDraw(ScreenScene* screenScene){
	if(!(SPI2->SR & SPI_SR_BSY)){		
		if(screenScene->currDrawedColumn == 0){
			if(screenScene->locked == 0){
			Swap(&(screenScene->currWaitFrame), &(screenScene->currDrawedFrame));
			}
		}
		GPIOA->BSRR = GPIO_BSRR_BS_8;
		
		SPI2->DR = screenScene->frame[screenScene->currDrawedFrame][screenScene->currDrawedColumn];
		
		(screenScene->currDrawedColumn)++;
		if(screenScene->currDrawedColumn > 7){
			screenScene->currDrawedColumn = 0;
		}
		GPIOA->BSRR = GPIO_BSRR_BR_8;
	}
}

void ScreenSet(ScreenScene* screenScene, ScreenObject screenObject[], uint8_t size){
	screenScene->locked = 1;
	Swap(&(screenScene->currWaitFrame), &(screenScene->currModifiedFrame));
	screenScene->locked = 0;
	memcpy(&(screenScene->frame[screenScene->currModifiedFrame]), &(screenScene->emptyFrame), 8*sizeof(uint16_t));
	for(uint8_t num = 0; num < size; num++){
		for(uint8_t x = 0; x < 8; x ++){
			if(((x +	screenObject[num].posX) >= 0) && ((x + screenObject[num].posX)<8)){
				uint16_t bufer = 0;
				if(screenObject[num].posY < 0){
					bufer = screenObject[num].templ[x] >> (~(screenObject[num].posY) + 1);
				}
				else{
					bufer = (uint16_t)(screenObject[num].templ[x] << (screenObject[num].posY));
				}
				screenScene->frame[screenScene->currModifiedFrame][x + screenObject[num].posX] |= (bufer << 8);
			}
		}
	}
}

void Swap(uint8_t* first, uint8_t* second){
	uint8_t buf = *first;
	*first = *second;
	*second = buf;
}
