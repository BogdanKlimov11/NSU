#ifndef AVALON_INTERFACE_H
#define AVALON_INTERFACE_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "cvidef.h"

int restrictIntValue(int* inputValue, int minValue, int maxValue);
int restrictDoubleValue(double* inputValue, double minValue, double maxValue);
int initAvalon(void);
int writeAvalon(int address, int subAddress, int inputValue);
int readAvalon(int address, int subAddress, int* outputValue);

#ifdef __cplusplus
    }
#endif

#endif