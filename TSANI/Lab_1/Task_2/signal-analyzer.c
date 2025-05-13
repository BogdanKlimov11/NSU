#include <analysis.h>
#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>

#include "toolbox.h"
#include "signal-analyzer.h"

#define SIGNAL_LENGTH 1000

static int panelHandle;

double amplitude = 5;
double frequency = 100;
double phase = 0;
double signalArray[SIGNAL_LENGTH], spectrumArray[SIGNAL_LENGTH];
double noiseLevel = 0;
double realComponents[1000], imagComponents[1000];

void createSignal()
{
    int index;

    for (index = 0; index < SIGNAL_LENGTH; index++)
    {
        signalArray[index] = amplitude * sin(2 * PI * frequency * index / 1000 + phase * PI) + Random(-noiseLevel, noiseLevel);
    }
}

void drawGraph()
{
    DeleteGraphPlot(PANEL, PANEL_GRAPH, -1, VAL_DELAYED_DRAW);
    SetAxisScalingMode(PANEL, PANEL_GRAPH, VAL_LEFT_YAXIS, VAL_MANUAL, -15, 15);
    SetAxisScalingMode(PANEL, PANEL_GRAPH, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0, SIGNAL_LENGTH);
    PlotY(PANEL, PANEL_GRAPH, signalArray, SIGNAL_LENGTH, VAL_DOUBLE, VAL_THIN_LINE, VAL_CONNECTED_POINTS, VAL_SOLID, 2, VAL_YELLOW);
}

void drawFft()
{
    int index;

    for (index = 0; index < 1000; index++)
    {
        realComponents[index] = signalArray[index];
        imagComponents[index] = 0.0;
    }

    FFT(realComponents, imagComponents, 1000);

    for (index = 0; index < 1000; index++)
    {
        realComponents[index] = abs(sqrt(pow(realComponents[index], 2) + pow(imagComponents[index], 2)));
    }
    DeleteGraphPlot(PANEL, PANEL_GRAPH_SPECTRUM, -1, VAL_DELAYED_DRAW);
    SetAxisScalingMode(PANEL, PANEL_GRAPH_SPECTRUM, VAL_LEFT_YAXIS, VAL_MANUAL, 0, 5000);
    SetAxisScalingMode(PANEL, PANEL_GRAPH_SPECTRUM, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0.0, 1000.0);
    PlotY(PANEL, PANEL_GRAPH_SPECTRUM, realComponents, 1000, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED);
}

int main(int argc, char *argv[])
{
    int error = 0, index;

    nullChk(InitCVIRTE(0, argv, 0));
    errChk(panelHandle = LoadPanel(0, "signal-analyzer.uir", PANEL));

    createSignal();
    drawGraph();
    drawFft();

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

int CVICALLBACK timerCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_TIMER_TICK)
    {
        createSignal();
        drawGraph();
        drawFft();
    }
    return 0;
}

int CVICALLBACK editAmplitude(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        GetCtrlVal(PANEL, PANEL_AMPLITUDE, &amplitude);
    }
    return 0;
}

int CVICALLBACK editFrequency(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        GetCtrlVal(PANEL, PANEL_FREQUENCY, &frequency);
    }
    return 0;
}

int CVICALLBACK editPhase(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        GetCtrlVal(PANEL, PANEL_PHASE, &phase);
    }
    return 0;
}

int CVICALLBACK editNoise(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        GetCtrlVal(PANEL, PANEL_NOISE, &noiseLevel);
    }
    return 0;
}

int CVICALLBACK saveButtonCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    int index;
    FILE *filePtr;
    if (event == EVENT_COMMIT)
    {
        filePtr = fopen("data.txt", "w");
        for (index = 0; index < 1000; index++)
        {
            fprintf(filePtr, "%i\t%f\n", index, signalArray[index]);
        }
        fclose(filePtr);
    }
    return 0;
}