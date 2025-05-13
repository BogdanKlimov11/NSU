#ifndef TRANSFER_FUNCTIONS_H
#define TRANSFER_FUNCTIONS_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "cvidef.h"

#define NADC_POINTS 1024
#define DAC_LSB_VALUE (3.3 / 255)
#define ADC_LSB_VALUE (2.56 / 1023)

int transferDac(int dacPanelHandle, double* inputData);
int transferAdc(int adcPanelHandle, int* inputData);
int calcDacOffsetError(double* inputData, double* resultValue);
int calcAdcOffsetError(int* inputData, double* resultValue);
int calcDacGainError(double* inputData, double* resultValue);
int calcAdcGainError(int* inputData, double* resultValue);
int calcDacIntegralNonlinearity(double* inputData, double* resultValue);
int calcAdcIntegralNonlinearity(int* inputData, double* resultValue);
int calcDacDifferentialNonlinearity(double* inputData, double* resultValue);
int calcAdcDifferentialNonlinearity(int* inputData, double* resultValue);

#ifdef __cplusplus
    }
#endif

#endif