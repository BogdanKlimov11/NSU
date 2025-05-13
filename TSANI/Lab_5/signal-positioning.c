#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>

#include "toolbox.h"
#include "signal-positioning.h"

#define NUM_SAMPLES 256
#define SHORT_SAMPLES 30
#define SAMPLING_FREQUENCY 60000000.0
#define SHORT_SAMPLING_FREQUENCY 10000000.0
#define TRIGGER_LEVEL 0.2
#define SHORT_TRIGGER_LEVEL 0.5
#define VOLTAGE_RANGE 2.0
#define SHORT_VOLTAGE_RANGE 5.0
#define PORT_VALUE 12
#define PXI_SLOT 5
#define COORD_SCALE 3.0 / 3.6 * 5.0

static int panelHandle;

double* countCoordinates(double* channel0Data, double* channel1Data, double* channel2Data, double* channel3Data)
{
    static double result[2];
    double u1 = 0.0, u2 = 0.0, u3 = 0.0, u4 = 0.0;
    for (int p = 0; p < NUM_SAMPLES; p++)
    {
        if (u1 < channel0Data[p])
            u1 = channel0Data[p];
        if (u2 < channel1Data[p])
            u2 = channel1Data[p];
        if (u3 < channel2Data[p])
            u3 = channel2Data[p];
        if (u4 < channel3Data[p])
            u4 = channel3Data[p];
    }
    result[0] = COORD_SCALE * (u2 + u3 - u1 - u4) / (u1 + u2 + u3 + u4);
    result[1] = COORD_SCALE * (u4 + u3 - u1 - u2) / (u1 + u2 + u3 + u4);
    return result;
}

void readChannels(int isShort, int* pointHandle, int* pointCount, double* xCoord, double* yCoord, double* xHistory, double* yHistory)
{
    double channel0Data[NUM_SAMPLES], channel1Data[NUM_SAMPLES], channel2Data[NUM_SAMPLES], channel3Data[NUM_SAMPLES];
    int sampleCount = isShort ? SHORT_SAMPLES : NUM_SAMPLES;
    scopeStartRead("0", channel0Data, sampleCount);
    scopeStartRead("1", channel1Data, sampleCount);
    scopeStartRead("2", channel2Data, sampleCount);
    scopeStartRead("3", channel3Data, sampleCount);
    DeleteGraphPlot(panelHandle, PANEL_GRAPH, -1, VAL_IMMEDIATE_DRAW);
    double* coords = countCoordinates(channel0Data, channel1Data, channel2Data, channel3Data);
    *xCoord = coords[0];
    *yCoord = coords[1];
    SetCtrlVal(panelHandle, PANEL_NUMERIC_X_COORD, *xCoord);
    SetCtrlVal(panelHandle, PANEL_NUMERIC_Y_COORD, *yCoord);
    PlotY(panelHandle, PANEL_GRAPH, channel0Data, sampleCount, VAL_DOUBLE, VAL_THIN_LINE, VAL_SOLID_SQUARE, VAL_SOLID, 1, VAL_RED);
    PlotY(panelHandle, PANEL_GRAPH, channel1Data, sampleCount, VAL_DOUBLE, VAL_THIN_LINE, VAL_SOLID_SQUARE, VAL_SOLID, 1, VAL_YELLOW);
    PlotY(panelHandle, PANEL_GRAPH, channel2Data, sampleCount, VAL_DOUBLE, VAL_THIN_LINE, VAL_SOLID_SQUARE, VAL_SOLID, 1, VAL_WHITE);
    PlotY(panelHandle, PANEL_GRAPH, channel3Data, sampleCount, VAL_DOUBLE, VAL_THIN_LINE, VAL_SOLID_SQUARE, VAL_SOLID, 1, VAL_GREEN);
    if (isShort)
    {
        xHistory[*pointCount] = *xCoord;
        yHistory[*pointCount] = *yCoord;
        double coordsToPlot[2] = {*xCoord, *yCoord};
        PlotStripChart(panelHandle, PANEL_SC, coordsToPlot, 2, 0, 0, VAL_DOUBLE);
        if (*pointCount == 0)
            PlotOval(panelHandle, PANEL_GRAPH_3, -3, -3, 3, 3, VAL_RED, VAL_TRANSPARENT);
        if (*pointCount != 0)
            DeleteGraphPlot(panelHandle, PANEL_GRAPH_3, *pointHandle, VAL_IMMEDIATE_DRAW);
        *pointHandle = PlotPoint(panelHandle, PANEL_GRAPH_3, *xCoord, *yCoord, VAL_SOLID_CIRCLE, VAL_YELLOW);
        DeleteGraphPlot(panelHandle, PANEL_GRAPH_2, -1, VAL_IMMEDIATE_DRAW);
        PlotY(panelHandle, PANEL_GRAPH_2, xHistory, NUM_SAMPLES, VAL_DOUBLE, VAL_THIN_LINE, VAL_SOLID_SQUARE, VAL_SOLID, 1, VAL_RED);
        PlotY(panelHandle, PANEL_GRAPH_2, yHistory, NUM_SAMPLES, VAL_DOUBLE, VAL_THIN_LINE, VAL_SOLID_SQUARE, VAL_SOLID, 1, VAL_YELLOW);
    }
}

void initPxi()
{
    scopeSlot(PXI_SLOT);
    scopeFrequency(NULL, SAMPLING_FREQUENCY, NUM_SAMPLES);
    scopeVertical("0", VOLTAGE_RANGE, SCOPE_50_OHM);
    scopeVertical("1", VOLTAGE_RANGE, SCOPE_50_OHM);
    scopeVertical("2", VOLTAGE_RANGE, SCOPE_50_OHM);
    scopeVertical("3", VOLTAGE_RANGE, SCOPE_50_OHM);
    scopeTrigger(NULL, TRIGGER_LEVEL, SCOPE_POSITIVE);
    portOut(0, 0);
    scopeStatus();
}

void separate()
{
    scopeSlot(PXI_SLOT);
    scopeFrequency("PFI1", SHORT_SAMPLING_FREQUENCY, SHORT_SAMPLES);
    scopeVertical("0", SHORT_VOLTAGE_RANGE, SCOPE_50_OHM);
    scopeVertical("1", SHORT_VOLTAGE_RANGE, SCOPE_50_OHM);
    scopeVertical("2", SHORT_VOLTAGE_RANGE, SCOPE_50_OHM);
    scopeVertical("3", SHORT_VOLTAGE_RANGE, SCOPE_50_OHM);
    scopeVertical("7", SHORT_VOLTAGE_RANGE, SCOPE_50_OHM);
    scopeTrigger("7", SHORT_TRIGGER_LEVEL, SCOPE_POSITIVE);
    portMask(0, 0xFF);
    portOut(0, PORT_VALUE);
    scopeStart(SHORT_SAMPLES);
    scopeStatus();
}

int main(int argc, char* argv[])
{
    int error = 0;
    nullChk(InitCVIRTE(0, argv, 0));
    errChk(panelHandle = LoadPanel(0, "signal-positioning.uir", PANEL));
    errChk(DisplayPanel(panelHandle));
    errChk(RunUserInterface());

Error:
    DiscardPanel(panelHandle);
    return 0;
}

int CVICALLBACK panelCallback(int panel, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_CLOSE)
        QuitUserInterface(0);
    return 0;
}

int CVICALLBACK initButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    static int isActive = 0;
    if (event == EVENT_LEFT_CLICK)
    {
        if (!isActive)
        {
            isActive = 1;
            initPxi();
        }
        else
        {
            isActive = 0;
            scopeClose();
        }
    }
    return 0;
}

int CVICALLBACK readTimerCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    static int isActive = 0;
    static int pointHandle = 0;
    static int pointCount = 0;
    static double xCoord = 0.0, yCoord = 0.0;
    static double xHistory[NUM_SAMPLES] = {0}, yHistory[NUM_SAMPLES] = {0};
    if (event == EVENT_TIMER_TICK && isActive)
        readChannels(0, &pointHandle, &pointCount, &xCoord, &yCoord, xHistory, yHistory);
    return 0;
}

int CVICALLBACK separateButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    static int isActive = 0;
    if (event == EVENT_LEFT_CLICK)
    {
        if (!isActive)
        {
            isActive = 1;
            separate();
        }
        else
        {
            isActive = 0;
            scopeClose();
        }
    }
    return 0;
}

int CVICALLBACK okButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    static int isProcessing = 0;
    if (event == EVENT_LEFT_CLICK)
    {
        isProcessing = !isProcessing;
        separate();
    }
    return 0;
}

int CVICALLBACK processTimerCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    static int isProcessing = 0;
    static int pointHandle = 0;
    static int pointCount = 0;
    static double xCoord = 0.0, yCoord = 0.0;
    static double xHistory[NUM_SAMPLES] = {0}, yHistory[NUM_SAMPLES] = {0};
    if (event == EVENT_TIMER_TICK && isProcessing)
    {
        readChannels(1, &pointHandle, &pointCount, &xCoord, &yCoord, xHistory, yHistory);
        if (pointCount < NUM_SAMPLES - 1)
            pointCount++;
        else
        {
            double xSum = 0.0, ySum = 0.0;
            for (int i = 0; i < NUM_SAMPLES; i++)
            {
                xSum += xHistory[i];
                ySum += yHistory[i];
            }
            xCoord = xSum / NUM_SAMPLES;
            yCoord = ySum / NUM_SAMPLES;
            SetCtrlVal(panelHandle, PANEL_NUMERIC_X_COORD, xCoord);
            SetCtrlVal(panelHandle, PANEL_NUMERIC_Y_COORD, yCoord);
            pointCount = 0;
            memset(xHistory, 0, sizeof(xHistory));
            memset(yHistory, 0, sizeof(yHistory));
        }
    }
    return 0;
}