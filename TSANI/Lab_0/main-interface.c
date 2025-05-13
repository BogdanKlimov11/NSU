#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>

#include "toolbox.h"
#include "main-interface.h"

static int panelHandle;

int main(int argc, char *argv[])
{
    int error = 0;
    
    nullChk(InitCVIRTE(0, argv, 0));
    errChk(panelHandle = LoadPanel(0, "main-interface.uir", PANEL));
    
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