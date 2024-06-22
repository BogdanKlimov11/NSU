#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>
#include "User Interface Application(TSANI).h"
#include "toolbox.h"

static int panelHandle;

int main (int argc, char *argv[])
{
    int error = 0;
    ni6251Slot(2);
    nullChk (InitCVIRTE (0, argv, 0));
    errChk (panelHandle = LoadPanel (0, "User Interface Application(TSANI).uir", PANEL));
    errChk (DisplayPanel (panelHandle));
    errChk (RunUserInterface ());
    ni6251Close();
    Error:
    DiscardPanel (panelHandle);
    return 0;
}

int CVICALLBACK panelCB (int panel, int event, void *callbackData,int eventData1, int eventData2)
{
    if (event == EVENT_CLOSE) QuitUserInterface (0);
    return 0;
}

int CVICALLBACK Measure (int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event)
    {
        case EVENT_COMMIT:
            double val;
            GetCtrlVal(PANEL, PANEL_NUMERICSLIDE, &val);
            analogOut(0, val);
            analogIn(0, &val);
            SetCtrlVal(PANEL, PANEL_NUMERICMETER, val);
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