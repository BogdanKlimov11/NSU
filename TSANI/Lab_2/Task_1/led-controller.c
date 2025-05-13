#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>

#include "toolbox.h"
#include "led-controller.h"

#define PORT_0 0
#define PORT_2 2
#define PORT_MASK_ALL 0xff
#define PORT_MASK_CONTROL 0x07
#define PORT_INITIAL_VALUE 0x00
#define PORT_CONTROL_VALUE 0x01

static int panelHandle;

int updateLeds()
{
    unsigned char portValue = 0;
    portIn(PORT_0, &portValue);
    SetCtrlVal(PANEL, PANEL_LED_0, 0x01 & (portValue >> 0));
    SetCtrlVal(PANEL, PANEL_LED_1, 0x01 & (portValue >> 1));
    SetCtrlVal(PANEL, PANEL_LED_2, 0x01 & (portValue >> 2));
    SetCtrlVal(PANEL, PANEL_LED_3, 0x01 & (portValue >> 3));
    SetCtrlVal(PANEL, PANEL_LED_4, 0x01 & (portValue >> 4));
    SetCtrlVal(PANEL, PANEL_LED_5, 0x01 & (portValue >> 5));
    SetCtrlVal(PANEL, PANEL_LED_6, 0x01 & (portValue >> 6));
    SetCtrlVal(PANEL, PANEL_LED_7, 0x01 & (portValue >> 7));
    return 0;
}

int cycleLeds()
{
    unsigned char portValue = 0;
    portIn(PORT_0, &portValue);
    portValue = (portValue >> 7) | (portValue << 1);
    portOut(PORT_0, portValue);
    updateLeds();
    return 0;
}

int main(int argc, char *argv[])
{
    int error = 0;
    unsigned char portValue = 0;
    ni6251Slot(2);

    portMask(PORT_2, PORT_MASK_CONTROL);
    portOut(PORT_2, PORT_CONTROL_VALUE);

    portMask(PORT_0, PORT_MASK_ALL);
    portOut(PORT_0, PORT_INITIAL_VALUE);

    nullChk(InitCVIRTE(0, argv, 0));
    errChk(panelHandle = LoadPanel(0, "led-controller.uir", PANEL));

    errChk(DisplayPanel(panelHandle));
    errChk(RunUserInterface());
    ni6251Close();

Error:
    DiscardPanel(panelHandle);
    return 0;
}

int CVICALLBACK panelCallback(int panel, int event, void *callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_CLOSE)
        QuitUserInterface(0);
    return 0;
}

int CVICALLBACK ledButtonCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    unsigned char portValue = 0;
    if (event == EVENT_COMMIT)
    {
        portIn(PORT_0, &portValue);
        switch (control)
        {
            case PANEL_BUTTON_0:
                portValue ^= (0x01 << 0);
                break;
            case PANEL_BUTTON_1:
                portValue ^= (0x01 << 1);
                break;
            case PANEL_BUTTON_2:
                portValue ^= (0x01 << 2);
                break;
            case PANEL_BUTTON_3:
                portValue ^= (0x01 << 3);
                break;
            case PANEL_BUTTON_4:
                portValue ^= (0x01 << 4);
                break;
            case PANEL_BUTTON_5:
                portValue ^= (0x01 << 5);
                break;
            case PANEL_BUTTON_6:
                portValue ^= (0x01 << 6);
                break;
            case PANEL_BUTTON_7:
                portValue ^= (0x01 << 7);
                break;
        }
        portOut(PORT_0, portValue);
        updateLeds();
    }
    return 0;
}

int CVICALLBACK runningLightCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    unsigned char portValue = 0;
    portIn(PORT_0, &portValue);
    portValue = (portValue >> 7) | (portValue << 1);
    portOut(PORT_0, portValue);
    updateLeds();
    return 0;
}

int CVICALLBACK toggleButtonCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    int toggleState;
    if (event == EVENT_COMMIT)
    {
        GetCtrlVal(PANEL, PANEL_TOGGLE_BUTTON, &toggleState);
        SetCtrlAttribute(PANEL, PANEL_TIMER, ATTR_ENABLED, toggleState);
    }
    return 0;
}