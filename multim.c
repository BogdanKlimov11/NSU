#include "multim.h"

Multim InitMultim(uint8_t buffersize, uint8_t initCountInterruptsWait){
	Multim multim;
	multim.bufferSize = buffersize;
	multim.countInterruptsWait = initCountInterruptsWait;
	multim.currCalc = 0;
	multim.currInterrupt = 0;
	multim.flagRunCalcAverAndWrite = 1;
	multim.halfBufferSize = buffersize / 2;
	return multim;
}

void CheckCalc(Multim* multim){
	if(multim->currInterrupt < multim->countInterruptsWait){
		multim->currInterrupt += 1;
	}
	if(multim->currInterrupt >= multim->countInterruptsWait){
		multim->currInterrupt = 0;
		multim->flagRunCalcAverAndWrite = 1;
	}
}

void CalcAver(Multim* multim, uint8_t BufferADC[], uint8_t templ[8]){
	if(multim->flagRunCalcAverAndWrite == 1){
		uint32_t summ = 0;
		
		for(uint8_t i = 0; i < multim->halfBufferSize; i++){
			summ += BufferADC[i + multim->halfBufferSize*multim->currCalc];
		}	
		
		uint8_t aver = (uint8_t)summ / multim->halfBufferSize;
		//uint8_t aver = 0xFF;
		
		WriteAver(aver, templ);
		
		multim->flagRunCalcAverAndWrite = 0;
	}
}

void WriteAver(uint8_t aver, uint8_t templ[8]){
	uint8_t bufer = (aver == 0x00) ? (0x00) : (uint8_t)( 0xFF << ( 0x07 - (aver >> 0x05) ) );
	for(uint8_t i = 0; i < 7; i++){
		templ[i] = templ[i+1];
	}
	//templ[7] = 0xFF;
	templ[7] = bufer;
}
