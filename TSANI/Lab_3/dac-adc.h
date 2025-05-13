#ifndef DAC_ADC_H
#define DAC_ADC_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "cvidef.h"

int initDac(void);
int initAdc(void);
int outputDacAdc(int dacNum, double outputValue);
int outputDacAdcCode(int dacNum, int outputCode);
int inputDacAdc(int adcNum, double* inputValue);
int inputDacAdcCode(int adcNum, int* inputValue);
int inputDacAdcIack(int adcNum, double* inputValue);
int setInterrupt(void);
int acknowledgeInterrupt(void);

#ifdef __cplusplus
    }
#endif

#endif