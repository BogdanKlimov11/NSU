#include <analysis.h>
#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>
#include "Sinus.h"
#include "toolbox.h"

static int panelHandle;

double A[1000];
double F[1000];
double U_0;
double v;

void CreatSignal(float U_0, float v)
{
    int t = 0;
    double R;
    double *pr;
    pr = &R;

    GetCtrlVal(panelHandle, PANEL_Noise, pr);
    for (t=0; t<1000; t++) {  A[t] = U_0*sin(2*3.14*v*t/1000)+Random(-R, R);}
}

void CreatSpectrum()
{
    double Y[1000]={0};
    double Ap[1000];
    int t=0;

    for (t=0; t<1000; t++)  {Ap[t] = A[t];}
    FFT(Ap, Y, 1000);
    for (t=0; t<1000; t++) { F[t]=sqrt(Ap[t]*Ap[t]+Y[t]*Y[t]); }
}

void DrowGraf_1()
{
    SetAxisScalingMode(panelHandle, PANEL_Graf1, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0, 1000);
    SetAxisScalingMode(panelHandle, PANEL_Graf1, VAL_LEFT_YAXIS, VAL_MANUAL, -15, 15);
    DeleteGraphPlot(panelHandle, PANEL_Graf1, -1, VAL_IMMEDIATE_DRAW);
    PlotY (panelHandle, PANEL_Graf1, A, 1000, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_GREEN);
}

void DrowGraf_2()
{
    SetAxisScalingMode(panelHandle, PANEL_Graf2, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0, 1000);
    DeleteGraphPlot(panelHandle, PANEL_Graf2, -1, VAL_IMMEDIATE_DRAW);
    PlotY (panelHandle, PANEL_Graf2, F, 1000, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_GREEN);
}

void Save()
{
    FILE *pF;
    pF = fopen("Save_graph.txt", "w");
    if (pF==NULL) printf("Can't open file") ;

    int i=0;

    for (i=0; i<1000; i++)
    {
        fprintf(pF, "%i 	 %f\n", i, A[i]);
    }

    fclose(pF);
}

int main (int argc, char *argv[])
{
    int error = 0;
    /* initialize and load resources */
    nullChk (InitCVIRTE (0, argv, 0));
    errChk (panelHandle = LoadPanel (0, "Sinus.uir", PANEL));

    /* display the panel and run the user interface */
    errChk (DisplayPanel (panelHandle));
    errChk (RunUserInterface ());

    Error:
    /* clean up */
    DiscardPanel (panelHandle);
    return 0;
}

/// HIFN Exit when the user dismisses the panel.
int CVICALLBACK panelCB (int panel, int event, void *callbackData,
                         int eventData1, int eventData2)
{
    if (event == EVENT_CLOSE)
        QuitUserInterface (0);
    return 0;
}

int CVICALLBACK Graf1 (int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
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

int CVICALLBACK Noise (int panel, int control, int event, void *callbackData, int eventData1, int eventData2) {
    switch (event) {
        case EVENT_COMMIT:
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

int CVICALLBACK Ampl (int panel, int control, int event, void *callbackData, int eventData1, int eventData2) {
    switch (event) {
        case EVENT_COMMIT:
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

int CVICALLBACK Freq (int panel, int control, int event, void *callbackData, int eventData1, int eventData2) {
    switch (event) {
        case EVENT_COMMIT:
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

int CVICALLBACK Timer1 (int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_TIMER_TICK:
            double *pa;
            pa = &U_0;
            double *pf;
            pf = &v;
            GetCtrlVal(panelHandle, PANEL_Ampl, pa);
            GetCtrlVal(panelHandle, PANEL_Freq, pf);

            CreatSignal(U_0, v);
            DrowGraf_1();

            CreatSpectrum();
            DrowGraf_2();

            break;
        case EVENT_DISCARD:
            break;
    }
    return 0;
}

int CVICALLBACK Save1 (int panel, int control, int event, void *callbackData, int eventData1, int eventData2) {
    switch (event) {
        case EVENT_COMMIT:
            Save();
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

int CVICALLBACK Graf2 (int panel, int control, int event, void *callbackData, int eventData1, int eventData2) {
    switch (event) {
        case EVENT_COMMIT:
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