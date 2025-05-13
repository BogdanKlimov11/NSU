#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>

#include "toolbox.h"
#include "avalon-interface.h"
#include "dac-adc.h"
#include "control-panel.h"
#include "transfer-functions.h"

int transferDac(int dacPanelHandle, double* inputData)
{
    int i;
    for (i = 0; i < 256; i++)
    {
        outputDacAdcCode(1, i);
        analogIn(0, (inputData + i));
        PlotStripChartPoint(dacPanelHandle, TAB_DAC_STRIP_CHART, *(inputData + i));
    }
    return 0;
}

int transferAdc(int adcPanelHandle, int* inputData)
{
    int i;
    for (i = 0; i < NADC_POINTS; i++)
    {
        analogOut(0, (2.56 / (NADC_POINTS - 1) * i));
        inputDacAdcCode(1, (inputData + i));
    }
    return 0;
}

int calcDacOffsetError(double* inputData, double* resultValue)
{
    *resultValue = *inputData;
    return 0;
}

int calcAdcOffsetError(int* inputData, double* resultValue)
{
    int i = 0;
    while (*(inputData + i) == 0)
        i++;
    *resultValue = -ADC_LSB_VALUE * i + *inputData;
    return 0;
}

int calcDacGainError(double* inputData, double* resultValue)
{
    *resultValue = *(inputData + 255) - 3.3;
    return 0;
}

int calcAdcGainError(int* inputData, double* resultValue)
{
    int i;
    double fullScaleError, offsetError;
    calcAdcOffsetError(inputData, &offsetError);
    for (i = 0; i < NADC_POINTS; i++)
    {
        if (*(inputData + i) == 1023)
            break;
    }
    fullScaleError = 2.56 * (1 - ((double)i / (NADC_POINTS - 1)));
    *resultValue = fullScaleError - offsetError;
    return 0;
}

int calcDacIntegralNonlinearity(double* inputData, double* resultValue)
{
    int i;
    double dataLine[256];
    double slope, intercept;
    double difference;
    *resultValue = 0;
    slope = (*(inputData + 255) - *inputData) / 256;
    intercept = *inputData;
    for (i = 0; i < 256; i++)
    {
        dataLine[i] = slope * i + intercept;
        difference = fabs(dataLine[i] - *(inputData + i));
        if (*resultValue < difference)
            *resultValue = difference;
    }
    return 0;
}

int calcAdcIntegralNonlinearity(int* inputData, double* resultValue)
{
    int i;
    double dataLine[NADC_POINTS];
    double slope, intercept;
    int difference;
    *resultValue = 0;
    slope = (double)(*(inputData + (NADC_POINTS - 1)) - *inputData) / NADC_POINTS;
    intercept = *inputData;
    for (i = 0; i < NADC_POINTS; i++)
    {
        dataLine[i] = slope * i + intercept;
        difference = fabs(dataLine[i] - *(inputData + i));
        if (*resultValue < difference)
            *resultValue = difference;
    }
    return 0;
}

int calcDacDifferentialNonlinearity(double* inputData, double* resultValue)
{
    int i;
    double difference;
    *resultValue = 0;
    for (i = 1; i < 256; i++)
    {
        difference = fabs(fabs(*(inputData + i) - *(inputData + i - 1)) - DAC_LSB_VALUE);
        if (*resultValue < difference)
            *resultValue = difference;
    }
    return 0;
}

int calcAdcDifferentialNonlinearity(int* inputData, double* resultValue)
{
    int i;
    double difference;
    *resultValue = 0;
    for (i = 1; i < NADC_POINTS; i++)
    {
        difference = fabs(fabs(*(inputData + i) - *(inputData + i - 1)) - 1);
        if (*resultValue < difference)
            *resultValue = difference;
    }
    return 0;
}