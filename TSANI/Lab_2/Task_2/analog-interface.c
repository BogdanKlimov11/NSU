#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>

#include "toolbox.h"
#include "analog-interface.h"

static int panelHandle;

int main(int argc, char *argv[])
{
    int error = 0;

    nullChk(InitCVIRTE(0, argv, 0));
    errChk(panelHandle = LoadPanel(0, "analog-interface.uir", PANEL));

    errChk(DisplayPanel(panelHandle));
    errChk(RunUserInterface());

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

int CVICALLBACK slideCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    double analogValue;
    if (event == EVENT_COMMIT)
    {
        GetCtrlVal(PANEL, PANEL_NUMERIC_SLIDE, &analogValue);
        analogOut(0, analogValue);
    }
    return 0;
}

int CVICALLBACK measureCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    double analogValue;
    if (event == EVENT_COMMIT)
    {
        analogIn(0, &analogValue);
        SetCtrlVal(PANEL, PANEL_NUMERIC_METER, analogValue);
    }
    return 0;
}