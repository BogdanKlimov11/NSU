#ifndef SIGNAL_FILTER_H
#define SIGNAL_FILTER_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "cvidef.h"

void calcLpfKernel(int window, int length, double cutoff, double* kernelData);
void spectralInversion(double* kernelData, int length);

#ifdef __cplusplus
    }
#endif

#endif