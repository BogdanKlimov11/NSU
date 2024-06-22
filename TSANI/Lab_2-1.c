#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>
#include "Lamp.h"
#include "toolbox.h"

static int panelHandle;

unsigned char temp = 0;
int run=1;

void Light (int number)
{
    portMask(0, 0xff);
    portIn(0, &temp);
    temp^=(1<<number);
    portOut(0, temp);
}

void Indet (unsigned char leg)
{
    SetCtrlVal (PANEL, PANEL_LED0, ((1<<0)==(leg&(1<<0)))?1:0);
    SetCtrlVal (PANEL, PANEL_LED1, ((1<<1)==(leg&(1<<1)))?1:0);
    SetCtrlVal (PANEL, PANEL_LED2, ((1<<2)==(leg&(1<<2)))?1:0);
    SetCtrlVal (PANEL, PANEL_LED3, ((1<<3)==(leg&(1<<3)))?1:0);
    SetCtrlVal (PANEL, PANEL_LED4, ((1<<4)==(leg&(1<<4)))?1:0);
    SetCtrlVal (PANEL, PANEL_LED5, ((1<<5)==(leg&(1<<5)))?1:0);
    SetCtrlVal (PANEL, PANEL_LED6, ((1<<6)==(leg&(1<<6)))?1:0);
    SetCtrlVal (PANEL, PANEL_LED7, ((1<<7)==(leg&(1<<7)))?1:0);
}

int main (int argc, char *argv[])
{
    int error = 0;
    nullChk (InitCVIRTE (0, argv, 0));
    errChk (panelHandle = LoadPanel (0, "Lamp.uir", PANEL));

    unsigned char temp = 0;
    ni6251Slot(2); // инициализация модуля
    // работа с модулем
    // переводим модуль в режим отображения состояния линий
    portMask(2, 0x07);
    portOut(2, 0x01);
    // гасим все светодиоды
    portMask(0, 0xff);
    portOut(0, 0x00);


    ni6251Close();   // деинициализация модуля

    errChk (DisplayPanel (panelHandle));
    errChk (RunUserInterface ());

    Error:
    DiscardPanel (panelHandle);
    return 0;
}

int CVICALLBACK panelCB (int panel, int event, void *callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_CLOSE)
        QuitUserInterface (0);
    return 0;
}

int CVICALLBACK RanL (int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event)
    {
        case EVENT_COMMIT:
            GetCtrlAttribute(panel, PANEL_TIMER, ATTR_ENABLED, &run);
            if (run==0)
            {
                run=1;
                SetCtrlAttribute(panel, PANEL_TIMER, ATTR_ENABLED, run);

            }
            else
            {
                run=0;
                SetCtrlAttribute(panel, PANEL_TIMER, ATTR_ENABLED, run);
            }
            break;
        case EVENT_LEFT_CLICK:
            break;
        case EVENT_RIGHT_CLICK:
            break;
        case EVENT_GOT_FOCUS:
            break;
        case EVENT_DISCARD:
            break;
    }
    return 0;
}

int CVICALLBACK Button (int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event)
    {
        case EVENT_COMMIT:
            switch (control)
            {
                case PANEL_COMMANDBUTTON_0:
                    Light(0);
                    break;
                case PANEL_COMMANDBUTTON_1:
                    Light(1);
                    break;
                case PANEL_COMMANDBUTTON_2:
                    Light(2);
                    break;
                case PANEL_COMMANDBUTTON_3:
                    Light(3);
                    break;
                case PANEL_COMMANDBUTTON_4:
                    Light(4);
                    break;
                case PANEL_COMMANDBUTTON_5:
                    Light(5);
                    break;
                case PANEL_COMMANDBUTTON_6:
                    Light(6);
                    break;
                case PANEL_COMMANDBUTTON_7:
                    Light(7);
                    break;
            }
            portIn(0, &temp);
            Indet(temp);
            break;
    }
    return 0;
}

int CVICALLBACK TIMER (int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event)
    {
        case EVENT_TIMER_TICK:
            unsigned char temp1 = 0;
            portMask(0, 0xff);
            portIn(0, &temp1);
            temp1 = (temp1 << 1)| (temp1 >> 7);
            portOut(0, temp1);
            portIn(0, &temp1);
            Indet(temp1);
            break;
        case EVENT_DISCARD:
            break;
    }
    return 0;
}