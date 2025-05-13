#include <analysis.h>
#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>

#include "toolbox.h"
#include "avalon-interface.h"
#include "dac-adc.h"
#include "control-panel.h"
#include "transfer-functions.h"

static int panelHandle;

int main(int argc, char* argv[])
{
    int error = 0;
    ni6251Slot(2);
    initAvalon();
    initDac();
    initAdc();
    nullChk(InitCVIRTE(0, argv, 0));
    errChk(panelHandle = LoadPanel(0, "control-panel.uir", PANEL));
    errChk(DisplayPanel(panelHandle));
    errChk(RunUserInterface());

Error:
    DiscardPanel(panelHandle);
    ni6251Close();
    return 0;
}

int CVICALLBACK panelCallback(int panel, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_CLOSE)
        QuitUserInterface(0);
    return 0;
}

int CVICALLBACK dacButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int dacPanelHandle, i;
        double inputData[256];
        double resultValue;
        FILE* pF;
        GetPanelHandleFromTabPage(panelHandle, PANEL_TAB_CONTROL, 0, &dacPanelHandle);
        ClearStripChart(dacPanelHandle, TAB_DAC_STRIP_CHART);
        SetCtrlVal(dacPanelHandle, TAB_DAC_OFFSET, 0.0);
        SetCtrlVal(dacPanelHandle, TAB_DAC_GAIN, 0.0);
        SetCtrlVal(dacPanelHandle, TAB_DAC_INTEG, 0.0);
        SetCtrlVal(dacPanelHandle, TAB_DAC_DIFF, 0.0);
        transferDac(dacPanelHandle, &inputData[0]);
        calcDacOffsetError(&inputData[0], &resultValue);
        SetCtrlVal(dacPanelHandle, TAB_DAC_OFFSET, (resultValue / DAC_LSB_VALUE));
        calcDacGainError(&inputData[0], &resultValue);
        SetCtrlVal(dacPanelHandle, TAB_DAC_GAIN, (resultValue / DAC_LSB_VALUE));
        calcDacIntegralNonlinearity(&inputData[0], &resultValue);
        SetCtrlVal(dacPanelHandle, TAB_DAC_INTEG, (resultValue / DAC_LSB_VALUE));
        calcDacDifferentialNonlinearity(&inputData[0], &resultValue);
        SetCtrlVal(dacPanelHandle, TAB_DAC_DIFF, (resultValue / DAC_LSB_VALUE));
        pF = fopen("DAC.txt", "w");
        for (i = 0; i < 256; i++)
        {
            fprintf(pF, "%f\n", inputData[i]);
        }
        fclose(pF);
    }
    return 0;
}

int CVICALLBACK dacLoadDataCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int dacPanelHandle, i;
        double inputData[256];
        double resultValue;
        FILE* pF;
        GetPanelHandleFromTabPage(panelHandle, PANEL_TAB_CONTROL, 0, &dacPanelHandle);
        ClearStripChart(dacPanelHandle, TAB_DAC_STRIP_CHART);
        SetCtrlVal(dacPanelHandle, TAB_DAC_OFFSET, 0.0);
        SetCtrlVal(dacPanelHandle, TAB_DAC_GAIN, 0.0);
        SetCtrlVal(dacPanelHandle, TAB_DAC_INTEG, 0.0);
        SetCtrlVal(dacPanelHandle, TAB_DAC_DIFF, 0.0);
        pF = fopen("DAC.txt", "r");
        if (pF != NULL)
        {
            for (i = 0; i < 256; i++)
            {
                fscanf(pF, "%lf", &inputData[i]);
                PlotStripChartPoint(dacPanelHandle, TAB_DAC_STRIP_CHART, inputData[i]);
            }
        }
        fclose(pF);
        calcDacOffsetError(&inputData[0], &resultValue);
        SetCtrlVal(dacPanelHandle, TAB_DAC_OFFSET, (resultValue / DAC_LSB_VALUE));
        calcDacGainError(&inputData[0], &resultValue);
        SetCtrlVal(dacPanelHandle, TAB_DAC_GAIN, (resultValue / DAC_LSB_VALUE));
        calcDacIntegralNonlinearity(&inputData[0], &resultValue);
        SetCtrlVal(dacPanelHandle, TAB_DAC_INTEG, (resultValue / DAC_LSB_VALUE));
        calcDacDifferentialNonlinearity(&inputData[0], &resultValue);
        SetCtrlVal(dacPanelHandle, TAB_DAC_DIFF, (resultValue / DAC_LSB_VALUE));
    }
    return 0;
}

int CVICALLBACK adcButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int adcPanelHandle, i, start, end;
        int inputData[NADC_POINTS];
        double resultValue;
        double inputVoltage[NADC_POINTS], difference[NADC_POINTS];
        double calibratedVoltage[NADC_POINTS], adcLsbCalibrated;
        FILE* pF;
        GetPanelHandleFromTabPage(panelHandle, PANEL_TAB_CONTROL, 1, &adcPanelHandle);
        DeleteGraphPlot(adcPanelHandle, TAB_ADC_GRAPH, -1, VAL_IMMEDIATE_DRAW);
        DeleteGraphPlot(adcPanelHandle, TAB_ADC_GRAPH_2, -1, VAL_IMMEDIATE_DRAW);
        SetCtrlVal(adcPanelHandle, TAB_ADC_OFFSET, 0.0);
        SetCtrlVal(adcPanelHandle, TAB_ADC_GAIN, 0.0);
        SetCtrlVal(adcPanelHandle, TAB_ADC_INTEG, 0.0);
        SetCtrlVal(adcPanelHandle, TAB_ADC_DIFF, 0.0);
        switch (control)
        {
            case TAB_ADC_ADC_BUTTON:
                transferAdc(adcPanelHandle, &inputData[0]);
                pF = fopen("ADC.txt", "w");
                for (i = 0; i < NADC_POINTS; i++)
                {
                    fprintf(pF, "%i\n", inputData[i]);
                    inputVoltage[i] = 2.56 / (NADC_POINTS - 1) * i;
                }
                fclose(pF);
                break;
            case TAB_ADC_ADC_BUTTON_2:
                pF = fopen("ADC.txt", "r");
                for (i = 0; i < NADC_POINTS; i++)
                {
                    fscanf(pF, "%i", &inputData[i]);
                    inputVoltage[i] = 2.56 / (NADC_POINTS - 1) * i;
                }
                fclose(pF);
                break;
        }
        PlotXY(adcPanelHandle, TAB_ADC_GRAPH, inputVoltage, inputData, NADC_POINTS, VAL_DOUBLE, VAL_INTEGER, VAL_SCATTER, VAL_SIMPLE_DOT, VAL_SOLID, 1, VAL_GREEN);
        difference[0] = 0;
        for (i = 1; i < NADC_POINTS; i++)
        {
            difference[i] = *(inputData + i) - *(inputData + i - 1) - 1024 / NADC_POINTS;
        }
        PlotXY(adcPanelHandle, TAB_ADC_GRAPH_2, inputVoltage, difference, NADC_POINTS, VAL_DOUBLE, VAL_DOUBLE, VAL_SCATTER, VAL_SIMPLE_DOT, VAL_SOLID, 1, VAL_RED);
        if (*inputData == 0)
        {
            for (start = 0; start < (NADC_POINTS - 1); start++)
                if (*(inputData + start) != 0)
                    break;
            start--;
        }
        else
            start = 0;
        start = *(inputData + start);
        for (end = 0; end < (NADC_POINTS - 1); end++)
            if (*(inputData + end) == 1023)
                break;
        end = *(inputData + end);
        adcLsbCalibrated = 2.56 / (double)(end - start);
        for (i = 0; i < NADC_POINTS; i++)
            calibratedVoltage[i] = *(inputData + i) * adcLsbCalibrated + start * ADC_LSB_VALUE;
        pF = fopen("debug.txt", "w");
        for (i = 0; i < NADC_POINTS; i++)
        {
            fprintf(pF, "%i\n", *(inputData + i));
        }
        fclose(pF);
        PlotXY(adcPanelHandle, TAB_ADC_GRAPH, calibratedVoltage, inputData, NADC_POINTS, VAL_DOUBLE, VAL_INTEGER, VAL_SCATTER, VAL_SIMPLE_DOT, VAL_SOLID, 1, VAL_RED);
        calcAdcOffsetError(&inputData[0], &resultValue);
        SetCtrlVal(adcPanelHandle, TAB_ADC_OFFSET, (resultValue / ADC_LSB_VALUE));
        calcAdcGainError(&inputData[0], &resultValue);
        SetCtrlVal(adcPanelHandle, TAB_ADC_GAIN, (resultValue / ADC_LSB_VALUE));
        calcAdcIntegralNonlinearity(&inputData[0], &resultValue);
        SetCtrlVal(adcPanelHandle, TAB_ADC_INTEG, resultValue);
        calcAdcDifferentialNonlinearity(&inputData[0], &resultValue);
        SetCtrlVal(adcPanelHandle, TAB_ADC_DIFF, resultValue);
    }
    return 0;
}

int CVICALLBACK loadUserInterfaceCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    switch (event)
    {
        case EVENT_TIMER_TICK:
            int dacPanelHandle;
            int adcPanelHandle;
            GetPanelHandleFromTabPage(panelHandle, PANEL_TAB_CONTROL, 0, &dacPanelHandle);
            GetPanelHandleFromTabPage(panelHandle, PANEL_TAB_CONTROL, 1, &adcPanelHandle);
            SetCtrlAttribute(dacPanelHandle, TAB_DAC_STRIP_CHART, ATTR_GRID_COLOR, VAL_TRANSPARENT);
            SetCtrlVal(dacPanelHandle, TAB_DAC_LSB, DAC_LSB_VALUE);
            SetCtrlVal(adcPanelHandle, TAB_ADC_LSB, ADC_LSB_VALUE);
            SetCtrlAttribute(dacPanelHandle, TAB_DAC_TIMER, ATTR_ENABLED, 0);
            break;
    }
    return 0;
}

int CVICALLBACK histButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int diffPanelHandle, defaultCode, code, sampleCount;
        int i, range, histogram[11], elementCount;
        double voltage, base, top, counts[11], histAxis[11], testValue, tempValue;
        unsigned char interruptStatus = 0;
        GetPanelHandleFromTabPage(panelHandle, PANEL_TAB_CONTROL, 2, &diffPanelHandle);
        DeleteGraphPlot(diffPanelHandle, TAB_DIFF_GRAPH, -1, VAL_IMMEDIATE_DRAW);
        GetCtrlVal(diffPanelHandle, TAB_DIFF_NUMERIC_U, &voltage);
        GetCtrlVal(diffPanelHandle, TAB_DIFF_NUMBER, &sampleCount);
        analogOut(0, voltage);
        double inputData[sampleCount];
        writeAvalon(BOARD_ADDRESS, START_ADDRESS_REG, 0x00);
        writeAvalon(BOARD_ADDRESS, END_ADDRESS_REG, (sampleCount - 1));
        writeAvalon(BOARD_ADDRESS, TIMER_REG, 0x00);
        writeAvalon(BOARD_ADDRESS, COMMAND_REG, COMMAND_START_IACK);
        while ((interruptStatus & INT3_BIT) != INT3_BIT)
            portIn(2, &interruptStatus);
        for (i = 0; i < sampleCount; i++)
        {
            readAvalon(BOARD_ADDRESS, VIN1_SUBADDRESS, &code);
            inputData[i] = (double)code;
        }
        base = inputData[0];
        top = inputData[0];
        elementCount = 1;
        for (i = 0; i < sampleCount; i++)
        {
            if (inputData[i] > top)
            {
                top = inputData[i];
                elementCount++;
            }
            if (inputData[i] < base)
            {
                base = inputData[i];
                elementCount++;
            }
        }
        Histogram(inputData, sampleCount, (base - 0.5), (top + 0.5), histogram, histAxis, elementCount);
        SetAxisRange(diffPanelHandle, TAB_DIFF_GRAPH, VAL_MANUAL, (base - 2), (top + 2), VAL_MANUAL, 0, sampleCount);
        PlotXY(diffPanelHandle, TAB_DIFF_GRAPH, histAxis, histogram, elementCount, VAL_DOUBLE, VAL_INTEGER, VAL_VERTICAL_BAR, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED);
        FILE* pF;
        pF = fopen("debug1.txt", "w");
        for (i = 0; i < elementCount; i++)
        {
            fprintf(pF, "%i\n", histogram[i]);
        }
        fclose(pF);
        pF = fopen("debug2.txt", "w");
        for (i = 0; i < elementCount; i++)
        {
            fprintf(pF, "%lf\n", histAxis[i]);
        }
        fclose(pF);
    }
    return 0;
}