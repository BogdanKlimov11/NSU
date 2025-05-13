#include <utility.h>
#include <userint.h>
#include <rs232.h>
#include <cvirte.h>
#include <ansi_c.h>

#include "toolbox.h"
#include "main-panel.h"
#include "scpi-interface.h"
#include "signal-filter.h"

#define WINDOW_RECTANGULAR 1
#define WINDOW_HAMMING 2
#define WINDOW_BLACKMAN 3
#define TWO_PI 6.283185307179586

void calcLpfKernel(int window, int length, double cutoff, double* kernelData)
{
    int i, m = length;
    double kernelSum = 0.0;
    switch (window)
    {
        case WINDOW_RECTANGULAR:
            for (i = 0; i < length; i++)
            {
                kernelData[i] = (i != m / 2) ? sin(TWO_PI * cutoff * (i - m / 2)) / ((i - m / 2) * TWO_PI * cutoff) : 1;
                kernelSum += kernelData[i];
            }
            break;
        case WINDOW_HAMMING:
            for (i = 0; i < length; i++)
            {
                kernelData[i] = (i != m / 2) ? sin(TWO_PI * cutoff * (i - m / 2)) / ((i - m / 2) * TWO_PI) * (0.54 - 0.46 * cos(TWO_PI * i / m)) : (2 * cutoff);
                kernelSum += kernelData[i];
            }
            break;
        case WINDOW_BLACKMAN:
            for (i = 0; i < length; i++)
            {
                kernelData[i] = (i != m / 2) ? sin(TWO_PI * cutoff * (i - m / 2)) / ((i - m / 2) * TWO_PI) * (0.42 - 0.5 * cos(TWO_PI * i / m) + 0.08 * cos(4 * TWO_PI * i / m)) : (2 * cutoff);
                kernelSum += kernelData[i];
            }
            break;
    }
    for (i = 0; i < length; i++)
        kernelData[i] /= kernelSum;
}

void spectralInversion(double* kernelData, int length)
{
    for (int i = 0; i < length; i++)
        kernelData[i] = (i != length / 2) ? (-kernelData[i]) : (1 - kernelData[i]);
}