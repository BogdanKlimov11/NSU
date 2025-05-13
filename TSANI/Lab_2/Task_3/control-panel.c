#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>

#include "toolbox.h"
#include "avalon-interface.h"
#include "dac-adc.h"
#include "control-panel.h"

static int panelHandle;

int main(int argc, char *argv[])
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

int CVICALLBACK panelCallback(int panel, int event, void *callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_CLOSE)
        QuitUserInterface(0);
    return 0;
}

int CVICALLBACK writeButtonCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    int boardAddress, subAddress, dataValue;
    if (event == EVENT_COMMIT)
    {
        GetCtrlVal(PANEL, PANEL_ADDRESS_WRITE, &boardAddress);
        GetCtrlVal(PANEL, PANEL_SUBADDRESS_WRITE, &subAddress);
        GetCtrlVal(PANEL, PANEL_DATA_WRITE, &dataValue);
        writeAvalon(boardAddress, subAddress, dataValue);
    }
    return 0;
}

int CVICALLBACK readButtonCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    int boardAddress, subAddress, dataValue;
    if (event == EVENT_COMMIT)
    {
        GetCtrlVal(PANEL, PANEL_ADDRESS_READ, &boardAddress);
        GetCtrlVal(PANEL, PANEL_SUBADDRESS_READ, &subAddress);
        readAvalon(boardAddress, subAddress, &dataValue);
        SetCtrlVal(PANEL, PANEL_DATA_READ, dataValue);
    }
    return 0;
}

int CVICALLBACK dac1SlideCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    double dataValue;
    if (event == EVENT_COMMIT)
    {
        GetCtrlVal(PANEL, PANEL_DAC1_SLIDE, &dataValue);
        outputDacAdc(1, dataValue);
    }
    return 0;
}

int CVICALLBACK dac2SlideCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    double dataValue;
    if (event == EVENT_COMMIT)
    {
        GetCtrlVal(PANEL, PANEL_DAC2_SLIDE, &dataValue);
        outputDacAdc(2, dataValue);
    }
    return 0;
}

int CVICALLBACK adc1MeasureCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    double dataValue;
    if (event == EVENT_COMMIT)
    {
        inputDacAdc(1, &dataValue);
        SetCtrlVal(PANEL, PANEL_ADC1_METER, dataValue);
    }
    return 0;
}

int CVICALLBACK adc2MeasureCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    double dataValue;
    if (event == EVENT_COMMIT)
    {
        inputDacAdcIack(2, &dataValue);
        SetCtrlVal(PANEL, PANEL_ADC2_METER, dataValue);
    }
    return 0;
}

int CVICALLBACK adcSetInterruptCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        setInterrupt();
    }
    return 0;
}

int CVICALLBACK adcIackCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        acknowledgeInterrupt();
    }
    return 0;
}