#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>
#include "3 (TSANI).h"
#include "toolbox.h"
#include "Bubluo.h"

static int panelHandle;
#define POINT 9000

void DCPerror()
{
    int t, codeACP;
    double val;
    double PercentError, LSBerror, error0, errorMash, errorInte, errorTemp, errorDuff;
    double Xarakt[256];
    double Real[256];
    double Analog[256];

    // разрядность - кол-во выходных уровней, отстающих друг от друга на один элементарный шаг 8 бит = 256
    for (t=0; t<256; t++) { Real[t] = 3.3*t/255; }

    for (t=0; t<256; t++)
    {
        Write(2,2,t);
        analogIn(0, &val);
        Xarakt[t] = val;
    }

    // характеристика преобразования (передаточная функция) - зависимость между сигналом на выходе ЦАПа и поданным на вход кодом
    PlotY (panelHandle, PANEL_GRAPH_1, Xarakt, 256, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_GREEN);
    PlotY (panelHandle, PANEL_GRAPH_1, Real, 256, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_BLUE);

    // вычисление ошибки нуля
    Write(2,2,0);
    analogIn(0, &val);
    error0=val;
    PercentError=error0/3.3*100;
    LSBerror=error0/3.3*256;
    SetCtrlVal(PANEL, PANEL_NUMERIC_0,  PercentError);
    SetCtrlVal(PANEL, PANEL_NUMERIC_2,  LSBerror);


    // убираем ошибку нуля
    for (t=0; t<256; t++)
    {
        Xarakt[t]=Xarakt[t]-error0;
    }

    // вычисленгие ошибки масштаба
    double temp=3.3;
    errorMash=Xarakt[255]-temp;
    PercentError=errorMash/3.3*100;
    if  (PercentError<0) PercentError=PercentError*(-1);
    LSBerror=errorMash/3.3*256;
    if  (LSBerror<0) LSBerror=LSBerror*(-1);
    SetCtrlVal(PANEL, PANEL_NUMERIC_1,  PercentError);
    SetCtrlVal(PANEL, PANEL_NUMERIC_3,  LSBerror);

    // убираем ошибку масштаба
    temp=255;
    for (t=0; t<256; t++)
    {
        Xarakt[t]=Xarakt[t]-(t/temp)*errorMash;
    }

    PlotY (panelHandle, PANEL_GRAPH_1, Xarakt, 256, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_RED);

    // вычисленгие интегральной нелин
    errorInte=0;
    for (t=0; t<256; t++)
    {
        errorTemp = Xarakt[t]- Real[t];
        if  (errorTemp<0)  errorTemp=errorTemp*(-1);
        if  (errorTemp>errorInte)  errorInte=errorTemp;
        Analog[t] = Xarakt[t] - Real[t];
    }
    PercentError=errorInte/3.3*100;
    LSBerror=errorInte/3.3*256;
    SetCtrlVal(PANEL, PANEL_NUMERIC_5,  PercentError);
    SetCtrlVal(PANEL, PANEL_NUMERIC_4,  LSBerror);

    // вычисление дифференциальной нелин
    errorDuff=0;
    double Duffer[255];
    for (t=0; t<255; t++)
    {
        errorTemp = (Xarakt[t] - Xarakt[t+1])- 3.3/256;
        Duffer[t] = errorTemp;
        if  (errorTemp<0)  errorTemp=errorTemp*(-1);
        if  (errorTemp>errorDuff)  errorDuff=errorTemp;
    }
    PlotY (panelHandle, PANEL_GRAPH_5, Duffer, 255, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_GREEN);

    PercentError=errorDuff/3.3*100;
    LSBerror=errorDuff/3.3*256;
    SetCtrlVal(PANEL, PANEL_NUMERIC_7,  PercentError);
    SetCtrlVal(PANEL, PANEL_NUMERIC_6,  LSBerror);

    // график аналогивых ошибок
    for (t=0; t<256; t++)
    {
        Analog[t] = Xarakt[t] - Real[t];
    }
    PlotY (panelHandle, PANEL_GRAPH_2, Analog, 256, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_GREEN);
}

void ACPread()
{
    FILE *DataACP;  int t = 0;
    int Xarakt[POINT];
    double val = 0;
    double Percent;

    DataACP = fopen ("Massiv.txt", "w");

    for (t=0; t<POINT; t++)
    {
        analogOut(0, val);
        val = val + 20 / pow(2, 16) ;
        Xarakt[t] = ACP();
        fprintf(DataACP, "%d\n", Xarakt[t]);

        Percent= 100 *(double)t/POINT;
        SetCtrlVal(PANEL, PANEL_NUMERICGAUGE, Percent);

    }
    fclose (DataACP);

}

void ACPerror()
{
    FILE *DataACP;
    int Xarakt[POINT]={0};
    int t = 0;
    int t_0=POINT-1;
    int t_1 = 0;
    int m=1;

    double PercentError, LSBerror, error0, errorMash, errorInte, errorTemp;
    DataACP = fopen ("Massiv.txt", "r");
    for (t=0; t<POINT; t++)
    {
        fscanf(DataACP, "%d\n", &Xarakt[t]);

        if ((Xarakt[t]!=0)&&(m==1))
        {
            t_1 = t-1;
            m=0;
        }

        if (Xarakt[t]==1023)
        {
            t_0=t;
            break;
        }
    }

    double Xarakt1[t_0+1];
    double XArray[t_0+1];
    for (t=0; t<t_0+1; t++)
    {
        Xarakt1[t]=Xarakt[t];
        XArray[t]=t*2.56/t_0;
    }

    fclose (DataACP);
    PlotXY (panelHandle, PANEL_GRAPH_4, XArray, Xarakt1, t_0+1, VAL_DOUBLE, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_YELLOW);

    // вычисление ошибки нуля
    error0 = (double)t_1/(double)(t_0-t_1);
    PercentError=error0*100;
    LSBerror=error0*1024;
    SetCtrlVal(PANEL, PANEL_NUMERIC_13,  PercentError);
    SetCtrlVal(PANEL, PANEL_NUMERIC_12,  LSBerror);

    // убираем ошибку нуля
    for (t=0; t<t_0+1; t++)
    {
        Xarakt1[t]=Xarakt1[t]+LSBerror;
    }

    // вычисленгие ошибки масштаба
    errorMash=Xarakt1[t_0]-1023;

    PercentError=errorMash/1023*100;
    if  (PercentError<0) PercentError=PercentError*(-1);
    LSBerror=errorMash;
    if  (LSBerror<0) LSBerror=LSBerror*(-1);
    SetCtrlVal(PANEL, PANEL_NUMERIC_11,  PercentError);
    SetCtrlVal(PANEL, PANEL_NUMERIC_10,  LSBerror);

    //убираем ошибку масштаба
    for (t=0; t<t_0+1; t++)
    {
        Xarakt1[t]=Xarakt1[t]-errorMash*(double)t/(double)t_0;
    }

    PlotXY (panelHandle, PANEL_GRAPH_4, XArray, Xarakt1, t_0+1, VAL_DOUBLE, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_GREEN);

    // вычисленгие интегральной нелин
    double Real[t_0+1];
    double Analog[t_0+1];

    for (t=0; t<t_0+1; t++)
    {
        Real[t]=(1023.0/(double)t_0)*t;
    }

    PlotXY (panelHandle, PANEL_GRAPH_4, XArray, Real, t_0+1, VAL_DOUBLE, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_BLUE);

    errorInte=0;
    for (t=0; t<t_0+1; t++)
    {
        if (t>t_1) { Analog[t] = Xarakt1[t]- Real[t]; }
        else { Analog[t]=0; }

        errorTemp = Analog[t];
        if  (errorTemp<0) { errorTemp=errorTemp*(-1); }
        if  (errorTemp>errorInte) {errorInte=errorTemp; }

    }

    PercentError=errorInte/1023*100;
    LSBerror=errorInte;
    SetCtrlVal(PANEL, PANEL_NUMERIC_9,  PercentError);
    SetCtrlVal(PANEL, PANEL_NUMERIC_8,  LSBerror);

    PlotXY (panelHandle, PANEL_GRAPH_3, XArray, Analog, t_0+1, VAL_DOUBLE, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_RED);
}

int main (int argc, char *argv[])
{
    int error = 0;
    ni6251Slot(2);
    nullChk (InitCVIRTE (0, argv, 0));
    errChk (panelHandle = LoadPanel (0, "3 (TSANI).uir", PANEL));
    errChk (DisplayPanel (panelHandle));
    Write(2,0,3);

//	  DCPerror();
//	  ACPread();
    ACPerror();

//    ACPerror();

    errChk (RunUserInterface ());
    Error:
    DiscardPanel (panelHandle);
    ni6251Close();
    return 0;
}

int CVICALLBACK panelCB (int panel, int event, void *callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_CLOSE)
        QuitUserInterface (0);
    return 0;
}