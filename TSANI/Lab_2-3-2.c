#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>
#include "User Interface Application(TSANI).h"
#include "toolbox.h"
#include "ourBubl.h"

static int panelHandle;
int value, AdresPlat, SybadressRegustr, SybadressDan;

int main (int argc, char *argv[])
{
    int error = 0;
    ni6251Slot(2);
    nullChk (InitCVIRTE (0, argv, 0));
    errChk (panelHandle = LoadPanel (0, "User Interface Application(TSANI).uir", PANEL));
    errChk (DisplayPanel (panelHandle));

    Write(2,0,3); // разрешение

    ////////////////////////////////
    errChk (RunUserInterface ());
    Error:
    DiscardPanel (panelHandle);

    ni6251Close();
    return 0;
}

int CVICALLBACK panelCB (int panel, int event, void *callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_CLOSE) QuitUserInterface (0);
    return 0;
}

int CVICALLBACK COMMANDBUTTON_READ (int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event)
    {
        case EVENT_COMMIT:
            GetCtrlVal(PANEL,PANEL_NUMERIC_Adress_plat_2, &AdresPlat);
            GetCtrlVal(PANEL,PANEL_NUMERIC_Sybadress_r_2, &SybadressRegustr);
            value=Read (AdresPlat, SybadressRegustr);
            SetCtrlVal(PANEL, PANEL_NUMERIC, value);   // вернуть прочитанные данные
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

int CVICALLBACK COMMANDBUTTON_WRITE (int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event)
    {
        case EVENT_COMMIT:
            GetCtrlVal(PANEL,PANEL_NUMERIC_Adress_plata,&AdresPlat);
            GetCtrlVal(PANEL,PANEL_NUMERIC_Sybadress_dan, &SybadressDan);
            GetCtrlVal(PANEL,PANEL_NUMERIC_Sybadress_reg, &SybadressRegustr);
            Write (AdresPlat, SybadressRegustr, SybadressDan);
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

int CVICALLBACK COMMANDBUTTON_U (int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    double U;
    switch (event)
    {
        case EVENT_COMMIT:
            GetCtrlVal(PANEL,PANEL_NUMERICSLIDE, &U);
            WriteDAC(U);
            value=Read (2, 2);
            U = value*3.3/255;
            SetCtrlVal(PANEL,PANEL_NUMERICMETER, U);
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

int CVICALLBACK COMMANDBUTTON_ACP (int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    int codeACP;
    switch (event)
    {
        case EVENT_COMMIT:
            double U;
            codeACP=ACP();
            U = codeACP*2.56/1024;
            SetCtrlVal(PANEL, PANEL_NUMERIC_2, U);
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

int CVICALLBACK COMMANDBUTTON_PRER (int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event)
    {
        case EVENT_COMMIT:
            Prer();

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

int CVICALLBACK COMMANDBUTTON_SNAT (int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event)
    {
        case EVENT_COMMIT:
            Snat();

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