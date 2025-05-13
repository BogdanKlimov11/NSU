#include <analysis.h>
#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>

#include "toolbox.h"
#include "signal-panel.h"

#define WAVEFORM_SINE 1
#define WAVEFORM_SQUARE 2
#define WAVEFORM_TRIANGLE 3
#define WAVEFORM_RAMP_UP 4
#define TRIGGER_LEVEL 0.001
#define SCOPE_RANGE_FACTOR 2.0
#define PLOT_POINTS 100
#define TWO_PI 6.283185307179586

static int panelHandle;

void initDevices(int genSlot, int scopeSlot)
{
    if (!fgenSlot(genSlot))
    {
        SetCtrlAttribute(panelHandle, PANEL_FGEN_SLOT, ATTR_CTRL_MODE, VAL_INDICATOR);
    }
    if (!scopeSlot(scopeSlot))
    {
        SetCtrlAttribute(panelHandle, PANEL_SCOPE_SLOT, ATTR_CTRL_MODE, VAL_INDICATOR);
    }
}

void startGeneration(double amplitude, double frequency, int waveformForm)
{
    switch (waveformForm)
    {
        case WAVEFORM_SINE:
            fgenStartStandartWaveForm(amplitude, frequency, FGEN_SINE);
            break;
        case WAVEFORM_SQUARE:
            fgenStartStandartWaveForm(amplitude, frequency, FGEN_SQUARE);
            break;
        case WAVEFORM_TRIANGLE:
            fgenStartStandartWaveForm(amplitude, frequency, FGEN_TRIANGLE);
            break;
        case WAVEFORM_RAMP_UP:
            fgenStartStandartWaveForm(amplitude, frequency, FGEN_RAMP_UP);
            break;
    }
}

void stopSignal()
{
    fgenStop();
    scopeClose();
}

void readPlotWaveform(double frequency, int numSamples, float range, double** waveformData, double** timeOrFrequency)
{
    *waveformData = (double*)malloc(numSamples * sizeof(double));
    *timeOrFrequency = (double*)malloc(numSamples * sizeof(double));
    if (*waveformData == NULL || *timeOrFrequency == NULL)
        return;

    for (int i = 0; i < numSamples; i++)
    {
        (*timeOrFrequency)[i] = i * (1.0 / frequency);
    }

    scopeFrequency(NULL, frequency, numSamples);
    scopeVertical("0", range, SCOPE_50_OHM);
    scopeTrigger("TRIG", TRIGGER_LEVEL, SCOPE_POSITIVE);
    scopeStartRead("0", *waveformData, numSamples);

    DeleteGraphPlot(panelHandle, PANEL_SIGNAL, -1, VAL_IMMEDIATE_DRAW);
    PlotXY(panelHandle, PANEL_SIGNAL, *timeOrFrequency, *waveformData, numSamples, VAL_DOUBLE, VAL_DOUBLE, VAL_THIN_LINE, VAL_SIMPLE_DOT, VAL_SOLID, PLOT_POINTS, VAL_GREEN);

    double* complexData = (double*)malloc(numSamples * sizeof(double));
    if (complexData == NULL)
    {
        free(*waveformData);
        free(*timeOrFrequency);
        return;
    }

    for (int i = 0; i < numSamples; i++)
    {
        complexData[i] = 0;
    }

    FFT(*waveformData, complexData, numSamples);

    for (int i = 0; i < numSamples; i++)
    {
        (*timeOrFrequency)[i] = TWO_PI / (*timeOrFrequency)[i];
    }

    DeleteGraphPlot(panelHandle, PANEL_FOURIER, -1, VAL_IMMEDIATE_DRAW);
    PlotXY(panelHandle, PANEL_FOURIER, *timeOrFrequency, *waveformData, numSamples, VAL_DOUBLE, VAL_DOUBLE, VAL_THIN_LINE, VAL_SIMPLE_DOT, VAL_SOLID, PLOT_POINTS, VAL_GREEN);

    free(complexData);
    free(*waveformData);
    free(*timeOrFrequency);
}

void impulseGeneration(int freqSteps, float freqMin, float freqMax, float stepDuration, double** freqList, double** timeIntervalList, int waveformForm, double amplitude)
{
    *freqList = (double*)malloc(freqSteps * sizeof(double));
    *timeIntervalList = (double*)malloc(freqSteps * sizeof(double));
    if (*freqList == NULL || *timeIntervalList == NULL)
        return;

    fgenGenerateFreqList(freqSteps, freqMin, freqMax, stepDuration, *freqList, *timeIntervalList);

    switch (waveformForm)
    {
        case WAVEFORM_SINE:
            fgenStart(FGEN_SINE, amplitude, freqSteps, *freqList, *timeIntervalList);
            break;
        case WAVEFORM_SQUARE:
            fgenStart(FGEN_SQUARE, amplitude, freqSteps, *freqList, *timeIntervalList);
            break;
        case WAVEFORM_TRIANGLE:
            fgenStart(FGEN_TRIANGLE, amplitude, freqSteps, *freqList, *timeIntervalList);
            break;
        case WAVEFORM_RAMP_UP:
            fgenStart(FGEN_RAMP_UP, amplitude, freqSteps, *freqList, *timeIntervalList);
            break;
    }

    free(*freqList);
    free(*timeIntervalList);
}

int main(int argc, char* argv[])
{
    int error = 0;
    nullChk(InitCVIRTE(0, argv, 0));
    errChk(panelHandle = LoadPanel(0, "signal-panel.uir", PANEL));
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

int CVICALLBACK generateCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int generateStatus;
        double signalAmplitude, signalFrequency;
        int waveformForm;
        GetCtrlVal(PANEL, PANEL_GENERATE, &generateStatus);
        if (generateStatus)
        {
            GetCtrlVal(PANEL, PANEL_FREQUENCY, &signalFrequency);
            GetCtrlVal(PANEL, PANEL_AMPLITUDE, &signalAmplitude);
            GetCtrlVal(PANEL, PANEL_WAVEFORM_FORM, &waveformForm);
            startGeneration(signalAmplitude, signalFrequency, waveformForm);
        }
        else
        {
            stopSignal();
        }
    }
    return 0;
}

int CVICALLBACK plotTimeCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_TIMER_TICK)
    {
        int generateStatus, impulseStatus, numSamples, waveformForm;
        double sampleRate, signalAmplitude, freqMax, freqMin, stepDuration;
        int freqSteps;
        double* waveformData = NULL;
        double* timeOrFrequency = NULL;
        GetCtrlVal(PANEL, PANEL_GENERATE, &generateStatus);
        if (generateStatus)
        {
            GetCtrlVal(PANEL, PANEL_SAMPLE_RATE, &sampleRate);
            GetCtrlVal(PANEL, PANEL_NUM_SAMPLES, &numSamples);
            GetCtrlVal(PANEL, PANEL_WAVEFORM_FORM, &waveformForm);
            GetCtrlVal(PANEL, PANEL_AMPLITUDE, &signalAmplitude);
            readPlotWaveform(sampleRate, numSamples, SCOPE_RANGE_FACTOR * signalAmplitude, &waveformData, &timeOrFrequency);
        }
        GetCtrlVal(PANEL, PANEL_IMPULSE_GENERATE, &impulseStatus);
        if (impulseStatus)
        {
            GetCtrlVal(PANEL, PANEL_FREQ_MAX, &freqMax);
            GetCtrlVal(PANEL, PANEL_FREQ_MIN, &freqMin);
            GetCtrlVal(PANEL, PANEL_STEP_DURATION, &stepDuration);
            GetCtrlVal(PANEL, PANEL_FREQ_STEPS, &freqSteps);
            GetCtrlVal(PANEL, PANEL_WAVEFORM_FORM, &waveformForm);
            GetCtrlVal(PANEL, PANEL_AMPLITUDE, &signalAmplitude);
            readPlotWaveform(10.0 / stepDuration, freqSteps, SCOPE_RANGE_FACTOR * signalAmplitude, &waveformData, &timeOrFrequency);
        }
    }
    return 0;
}

int CVICALLBACK slotCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int scopeSlot, fgenSlot;
        GetCtrlVal(PANEL, PANEL_SCOPE_SLOT, &scopeSlot);
        GetCtrlVal(PANEL, PANEL_FGEN_SLOT, &fgenSlot);
        initDevices(fgenSlot, scopeSlot);
    }
    return 0;
}

int CVICALLBACK changeCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        double signalAmplitude, signalFrequency;
        GetCtrlVal(PANEL, PANEL_FREQUENCY, &signalFrequency);
        GetCtrlVal(PANEL, PANEL_AMPLITUDE, &signalAmplitude);
        fgenAmplitude(signalAmplitude);
        Delay(1);
        fgenFrequency(signalFrequency);
    }
    return 0;
}

int CVICALLBACK impulseGenerateCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int impulseStatus, waveformForm, freqSteps;
        double signalAmplitude, freqMax, freqMin, stepDuration;
        double* freqList = NULL;
        double* timeIntervalList = NULL;
        GetCtrlVal(PANEL, PANEL_IMPULSE_GENERATE, &impulseStatus);
        if (impulseStatus)
        {
            GetCtrlVal(PANEL, PANEL_FREQ_MAX, &freqMax);
            GetCtrlVal(PANEL, PANEL_FREQ_MIN, &freqMin);
            GetCtrlVal(PANEL, PANEL_STEP_DURATION, &stepDuration);
            GetCtrlVal(PANEL, PANEL_FREQ_STEPS, &freqSteps);
            GetCtrlVal(PANEL, PANEL_WAVEFORM_FORM, &waveformForm);
            GetCtrlVal(PANEL, PANEL_AMPLITUDE, &signalAmplitude);
            impulseGeneration(freqSteps, freqMin, freqMax, stepDuration, &freqList, &timeIntervalList, waveformForm, signalAmplitude);
        }
        else
        {
            stopSignal();
        }
    }
    return 0;
}