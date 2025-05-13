#include <formatio.h>
#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>

#include "toolbox.h"
#include "i2c-interface.h"
#include "main-panel.h"

static int panelHandle;

#define TESTER_ADDRESS 0x01
#define LED_CONTROL_REG 0x00
#define READ_REG_BASE 0x08

int main(int argc, char* argv[])
{
    int error = 0;
    ni6251Slot(2);
    initI2c();
    nullChk(InitCVIRTE(0, argv, 0));
    errChk(panelHandle = LoadPanel(0, "main-panel.uir", PANEL));
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

int CVICALLBACK singleButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        double temperatureValue;
        readTemperature(&temperatureValue);
        SetCtrlVal(panelHandle, PANEL_NUMERIC_TEMP, temperatureValue);
    }
    return 0;
}

int CVICALLBACK temperatureTimerCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_TIMER_TICK)
    {
        double temperatureValue;
        readTemperature(&temperatureValue);
        SetCtrlVal(panelHandle, PANEL_NUMERIC_TEMP, temperatureValue);
        PlotStripChartPoint(panelHandle, PANEL_STRIP_CHART, temperatureValue);
    }
    return 0;
}

int CVICALLBACK temperatureTbuttonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int stateValue;
        GetCtrlVal(PANEL, PANEL_TBUTTON_TEMPERATURE, &stateValue);
        if (stateValue == 0x01)
            ClearStripChart(PANEL, PANEL_STRIP_CHART);
        SetCtrlAttribute(PANEL, PANEL_TIMER_TEMPERATURE, ATTR_ENABLED, stateValue);
    }
    return 0;
}

int CVICALLBACK ledButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_LEFT_CLICK)
    {
        int tempValue, ledData = 0;
        GetCtrlVal(PANEL, PANEL_LED_0, &tempValue);
        ledData |= (tempValue << 0);
        GetCtrlVal(PANEL, PANEL_LED_1, &tempValue);
        ledData |= (tempValue << 1);
        GetCtrlVal(PANEL, PANEL_LED_2, &tempValue);
        ledData |= (tempValue << 2);
        GetCtrlVal(PANEL, PANEL_LED_3, &tempValue);
        ledData |= (tempValue << 3);
        GetCtrlVal(PANEL, PANEL_LED_4, &tempValue);
        ledData |= (tempValue << 4);
        GetCtrlVal(PANEL, PANEL_LED_5, &tempValue);
        ledData |= (tempValue << 5);
        GetCtrlVal(PANEL, PANEL_LED_6, &tempValue);
        ledData |= (tempValue << 6);
        GetCtrlVal(PANEL, PANEL_LED_7, &tempValue);
        ledData |= (tempValue << 7);
        switch (control)
        {
            case PANEL_LED_0:
                ledData ^= (0x01 << 0);
                SetCtrlVal(PANEL, PANEL_LED_0, ((ledData >> 0) & 0x01));
                break;
            case PANEL_LED_1:
                ledData ^= (0x01 << 1);
                SetCtrlVal(PANEL, PANEL_LED_1, ((ledData >> 1) & 0x01));
                break;
            case PANEL_LED_2:
                ledData ^= (0x01 << 2);
                SetCtrlVal(PANEL, PANEL_LED_2, ((ledData >> 2) & 0x01));
                break;
            case PANEL_LED_3:
                ledData ^= (0x01 << 3);
                SetCtrlVal(PANEL, PANEL_LED_3, ((ledData >> 3) & 0x01));
                break;
            case PANEL_LED_4:
                ledData ^= (0x01 << 4);
                SetCtrlVal(PANEL, PANEL_LED_4, ((ledData >> 4) & 0x01));
                break;
            case PANEL_LED_5:
                ledData ^= (0x01 << 5);
                SetCtrlVal(PANEL, PANEL_LED_5, ((ledData >> 5) & 0x01));
                break;
            case PANEL_LED_6:
                ledData ^= (0x01 << 6);
                SetCtrlVal(PANEL, PANEL_LED_6, ((ledData >> 6) & 0x01));
                break;
            case PANEL_LED_7:
                ledData ^= (0x01 << 7);
                SetCtrlVal(PANEL, PANEL_LED_7, ((ledData >> 7) & 0x01));
                break;
        }
        writeWord(TESTER_ADDRESS, LED_CONTROL_REG, ledData);
    }
    return 0;
}

int CVICALLBACK ledTimerCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_TIMER_TICK)
    {
        int tempValue, ledData = 0;
        GetCtrlVal(PANEL, PANEL_LED_0, &tempValue);
        ledData |= (tempValue << 0);
        GetCtrlVal(PANEL, PANEL_LED_1, &tempValue);
        ledData |= (tempValue << 1);
        GetCtrlVal(PANEL, PANEL_LED_2, &tempValue);
        ledData |= (tempValue << 2);
        GetCtrlVal(PANEL, PANEL_LED_3, &tempValue);
        ledData |= (tempValue << 3);
        GetCtrlVal(PANEL, PANEL_LED_4, &tempValue);
        ledData |= (tempValue << 4);
        GetCtrlVal(PANEL, PANEL_LED_5, &tempValue);
        ledData |= (tempValue << 5);
        GetCtrlVal(PANEL, PANEL_LED_6, &tempValue);
        ledData |= (tempValue << 6);
        GetCtrlVal(PANEL, PANEL_LED_7, &tempValue);
        ledData |= (tempValue << 7);
        ledData = ((ledData >> 7) | (ledData << 1)) & 0xff;
        SetCtrlVal(PANEL, PANEL_NUMERIC_DEBUG, ledData);
        writeWord(TESTER_ADDRESS, LED_CONTROL_REG, ledData);
        SetCtrlVal(PANEL, PANEL_LED_0, ((ledData >> 0) & 0x01));
        SetCtrlVal(PANEL, PANEL_LED_1, ((ledData >> 1) & 0x01));
        SetCtrlVal(PANEL, PANEL_LED_2, ((ledData >> 2) & 0x01));
        SetCtrlVal(PANEL, PANEL_LED_3, ((ledData >> 3) & 0x01));
        SetCtrlVal(PANEL, PANEL_LED_4, ((ledData >> 4) & 0x01));
        SetCtrlVal(PANEL, PANEL_LED_5, ((ledData >> 5) & 0x01));
        SetCtrlVal(PANEL, PANEL_LED_6, ((ledData >> 6) & 0x01));
        SetCtrlVal(PANEL, PANEL_LED_7, ((ledData >> 7) & 0x01));
    }
    return 0;
}

int CVICALLBACK ledTbuttonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int stateValue;
        GetCtrlVal(PANEL, PANEL_TBUTTON_LED, &stateValue);
        SetCtrlAttribute(PANEL, PANEL_TIMER_LED, ATTR_ENABLED, stateValue);
    }
    return 0;
}

int CVICALLBACK wordButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int i;
        char outputData[9];
        int tempValue = 0;
        for (i = 0; i < 8; i++)
        {
            readWord(TESTER_ADDRESS, (READ_REG_BASE + i), &tempValue, 1);
            outputData[i] = (unsigned char)tempValue;
        }
        outputData[8] = '\0';
        SetCtrlVal(panelHandle, PANEL_STRING, outputData);
    }
    return 0;
}