#include <analysis.h>
#include <rs232.h>
#include <formatio.h>
#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>

#include "toolbox.h"
#include "signal-filter.h"
#include "scpi-interface.h"
#include "main-panel.h"

#define SAMPLING_RATE 1000000
#define NUM_SAMPLES (SAMPLING_RATE / 10)
#define TRIGGER_LEVEL 0.1
#define MAX_FREQUENCY 20000.0
#define FREQUENCY_SCALE 10.0
#define CUTOFF_SCALE 0.01
#define WINDOW_RECTANGULAR 1
#define WINDOW_HAMMING 2
#define WINDOW_BLACKMAN 3
#define TWO_PI 6.283185307179586

static int panelHandle;

int main(int argc, char* argv[])
{
    int error = 0;
    ni6251Slot(2);
    nullChk(InitCVIRTE(0, argv, 0));
    errChk(panelHandle = LoadPanel(0, "main-panel.uir", PANEL));
    errChk(DisplayPanel(panelHandle));
    errChk(RunUserInterface());

Error:
    DiscardPanel(panelHandle);
    ni6251Close();
    comDeinit(PORT_NUMBER);
    return 0;
}

int CVICALLBACK panelCallback(int panel, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_CLOSE)
        QuitUserInterface(0);
    return 0;
}

int CVICALLBACK waveformSelectCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int generatorState;
        SetCtrlVal(PANEL, PANEL_RADIOBUTTON_SINE, 0);
        SetCtrlVal(PANEL, PANEL_RADIOBUTTON_SQUARE, 0);
        SetCtrlVal(PANEL, PANEL_RADIOBUTTON_TRIANGLE, 0);
        SetCtrlVal(PANEL, control, 1);
        GetCtrlVal(PANEL, PANEL_TBUTTON_GENERATOR, &generatorState);
        if (generatorState)
        {
            switch (control)
            {
                case PANEL_RADIOBUTTON_SINE:
                    comWriteInt(":FUNCtion:WAVeform", 1);
                    break;
                case PANEL_RADIOBUTTON_SQUARE:
                    comWriteInt(":FUNCtion:WAVeform", 3);
                    break;
                case PANEL_RADIOBUTTON_TRIANGLE:
                    comWriteInt(":FUNCtion:WAVeform", 2);
                    break;
            }
        }
    }
    return 0;
}

int CVICALLBACK tbuttonInitCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int state;
        GetCtrlVal(PANEL, PANEL_TBUTTON_INIT, &state);
        if (!state)
        {
            comGeneratorOff(PORT_NUMBER);
            comDeinit(PORT_NUMBER);
            SetCtrlVal(PANEL, PANEL_STRING_STATE, "Port is closed");
            SetCtrlAttribute(PANEL, PANEL_TEXTMSG_STEP2, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_TEXTMSG_WAVEFORM, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_TEXTMSG_FREQUENCY, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_TEXTMSG_AMPLITUDE, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_TEXTMSG_DC_OFFSET, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_RADIOBUTTON_SINE, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_RADIOBUTTON_SQUARE, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_RADIOBUTTON_TRIANGLE, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_NUMERIC_FREQUENCY, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_NUMERIC_AMPLITUDE, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_NUMERIC_DC_OFFSET, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_TBUTTON_GENERATOR, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_TEXTMSG_STEP3, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_TEXTMSG_MOD_FREQUENCY, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_TEXTMSG_DEVIATION, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_NUMERIC_MOD_FREQUENCY, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_NUMERIC_DEVIATION, ATTR_DIMMED, 1);
            SetCtrlAttribute(PANEL, PANEL_TBUTTON_MODULATION, ATTR_DIMMED, 1);
            SetCtrlVal(PANEL, PANEL_TBUTTON_GENERATOR, 0);
            SetCtrlVal(PANEL, PANEL_TBUTTON_MODULATION, 0);
        }
        else
        {
            if (comInit(PORT_NUMBER, TIMEOUT) < 0)
            {
                SetCtrlVal(PANEL, PANEL_TBUTTON_INIT, 0);
            }
            else
            {
                SetCtrlVal(PANEL, PANEL_STRING_STATE, "Port is open");
                SetCtrlAttribute(PANEL, PANEL_TEXTMSG_STEP2, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_TEXTMSG_WAVEFORM, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_TEXTMSG_FREQUENCY, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_TEXTMSG_AMPLITUDE, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_TEXTMSG_DC_OFFSET, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_RADIOBUTTON_SINE, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_RADIOBUTTON_SQUARE, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_RADIOBUTTON_TRIANGLE, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_NUMERIC_FREQUENCY, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_NUMERIC_AMPLITUDE, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_NUMERIC_DC_OFFSET, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_TBUTTON_GENERATOR, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_TEXTMSG_STEP3, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_TEXTMSG_MOD_FREQUENCY, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_TEXTMSG_DEVIATION, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_NUMERIC_MOD_FREQUENCY, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_NUMERIC_DEVIATION, ATTR_DIMMED, 0);
                SetCtrlAttribute(PANEL, PANEL_TBUTTON_MODULATION, ATTR_DIMMED, 0);
            }
        }
    }
    return 0;
}

int CVICALLBACK frequencyCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        double value;
        int generatorState;
        GetCtrlVal(PANEL, PANEL_TBUTTON_GENERATOR, &generatorState);
        if (generatorState)
        {
            GetCtrlVal(PANEL, PANEL_NUMERIC_FREQUENCY, &value);
            comWriteDouble(":FREQuency", value);
        }
    }
    return 0;
}

int CVICALLBACK amplitudeCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        double value;
        int generatorState;
        GetCtrlVal(PANEL, PANEL_TBUTTON_GENERATOR, &generatorState);
        if (generatorState)
        {
            GetCtrlVal(PANEL, PANEL_NUMERIC_AMPLITUDE, &value);
            comWriteDouble(":AMPLitude:VOLTage", value);
        }
    }
    return 0;
}

int CVICALLBACK dcOffsetCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        double value;
        int generatorState;
        GetCtrlVal(PANEL, PANEL_TBUTTON_GENERATOR, &generatorState);
        if (generatorState)
        {
            GetCtrlVal(PANEL, PANEL_NUMERIC_DC_OFFSET, &value);
            comWriteDouble(":OFFSet", value);
        }
    }
    return 0;
}

int CVICALLBACK generatorCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        double value;
        int valueInt, generatorState, modulationState;
        GetCtrlVal(PANEL, PANEL_TBUTTON_GENERATOR, &generatorState);
        GetCtrlVal(PANEL, PANEL_TBUTTON_MODULATION, &modulationState);
        if (generatorState)
        {
            GetCtrlVal(PANEL, PANEL_RADIOBUTTON_SINE, &valueInt);
            if (valueInt)
                comWriteInt(":FUNCtion:WAVeform", 1);
            GetCtrlVal(PANEL, PANEL_RADIOBUTTON_TRIANGLE, &valueInt);
            if (valueInt)
                comWriteInt(":FUNCtion:WAVeform", 2);
            GetCtrlVal(PANEL, PANEL_RADIOBUTTON_SQUARE, &valueInt);
            if (valueInt)
                comWriteInt(":FUNCtion:WAVeform", 3);
            GetCtrlVal(PANEL, PANEL_NUMERIC_DC_OFFSET, &value);
            comWriteDouble(":OFFSet", value);
            GetCtrlVal(PANEL, PANEL_NUMERIC_FREQUENCY, &value);
            comWriteDouble(":FREQuency", value);
            GetCtrlVal(PANEL, PANEL_NUMERIC_AMPLITUDE, &value);
            comWriteDouble(":AMPLitude:VOLTage", value);
            if (modulationState)
            {
                GetCtrlVal(PANEL, PANEL_NUMERIC_MOD_FREQUENCY, &value);
                comWriteDouble(":SOURce:MODFM:RATe", value);
                GetCtrlVal(PANEL, PANEL_NUMERIC_DEVIATION, &value);
                comWriteDouble(":SOURce:MODFM:SPAN", value);
            }
        }
        else
        {
            comGeneratorOff(PORT_NUMBER);
        }
    }
    return 0;
}

int CVICALLBACK modFrequencyCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        double value;
        int modulationState;
        GetCtrlVal(PANEL, PANEL_TBUTTON_MODULATION, &modulationState);
        if (modulationState)
        {
            GetCtrlVal(PANEL, PANEL_NUMERIC_MOD_FREQUENCY, &value);
            comWriteDouble(":SOURce:MODFM:RATe", value);
        }
    }
    return 0;
}

int CVICALLBACK deviationCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        double value;
        int modulationState;
        GetCtrlVal(PANEL, PANEL_TBUTTON_MODULATION, &modulationState);
        if (modulationState)
        {
            GetCtrlVal(PANEL, PANEL_NUMERIC_DEVIATION, &value);
            comWriteDouble(":SOURce:MODFM:SPAN", value);
        }
    }
    return 0;
}

int CVICALLBACK modulationCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        double value;
        int generatorState, modulationState;
        GetCtrlVal(PANEL, PANEL_TBUTTON_GENERATOR, &generatorState);
        GetCtrlVal(PANEL, PANEL_TBUTTON_MODULATION, &modulationState);
        if (modulationState && generatorState)
        {
            comWriteDouble(":SOURce:SOURce", 0);
            comWriteDouble(":SOURce:WAVeform", 1);
            GetCtrlVal(PANEL, PANEL_NUMERIC_MOD_FREQUENCY, &value);
            comWriteDouble(":SOURce:MODFM:RATe", value);
            GetCtrlVal(PANEL, PANEL_NUMERIC_DEVIATION, &value);
            comWriteDouble(":SOURce:MODFM:SPAN", value);
            comWriteDouble(":SOURce:STATe", 2);
        }
        else if (!modulationState)
        {
            comWriteDouble(":SOURce:STATe", 0);
        }
    }
    return 0;
}

int CVICALLBACK timerCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_TIMER_TICK)
    {
        int n, ai0PanelHandle, i, activeTab;
        double df, max;
        double signalData[NUM_SAMPLES], signalXData[NUM_SAMPLES], signalFftData[NUM_SAMPLES];
        GetActiveTabPage(PANEL, PANEL_TABCONTROL, &activeTab);
        if (activeTab != 0)
            return 0;
        GetPanelHandleFromTabPage(PANEL, PANEL_TABCONTROL, 0, &ai0PanelHandle);
        analogInClk(NULL, SAMPLING_RATE);
        analogInTrigger("APFI0", TRIGGER_LEVEL);
        analogRead(0, signalData, NUM_SAMPLES, &n);
        DeleteGraphPlot(ai0PanelHandle, TAB_AI0_GRAPH_SIGNAL, -1, VAL_IMMEDIATE_DRAW);
        PlotY(ai0PanelHandle, TAB_AI0_GRAPH_SIGNAL, signalData, n, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_YELLOW);
        AutoPowerSpectrum(signalData, n, 1.0 / SAMPLING_RATE, signalFftData, &df);
        max = 0;
        for (i = 0; i < n; i++)
        {
            if (signalFftData[i] > max)
                max = signalFftData[i];
            signalXData[i] = i * FREQUENCY_SCALE;
        }
        for (i = 0; i < n; i++)
            signalFftData[i] /= max;
        DeleteGraphPlot(ai0PanelHandle, TAB_AI0_GRAPH_FFT, -1, VAL_DELAYED_DRAW);
        SetAxisScalingMode(ai0PanelHandle, TAB_AI0_GRAPH_FFT, VAL_LEFT_YAXIS, VAL_MANUAL, 0, 1);
        SetAxisScalingMode(ai0PanelHandle, TAB_AI0_GRAPH_FFT, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0.0, MAX_FREQUENCY);
        PlotXY(ai0PanelHandle, TAB_AI0_GRAPH_FFT, signalXData, signalFftData, n, VAL_DOUBLE, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED);
    }
    return 0;
}

int CVICALLBACK pfCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int pfPanelHandle;
        GetPanelHandleFromTabPage(PANEL, PANEL_TABCONTROL, 1, &pfPanelHandle);
        SetCtrlVal(pfPanelHandle, TAB_PF_RBUTTON_LPF, 0);
        SetCtrlVal(pfPanelHandle, TAB_PF_RBUTTON_HPF, 0);
        SetCtrlVal(pfPanelHandle, control, 1);
        lpfCallback(panel, control, event, callbackData, eventData1, eventData2);
    }
    return 0;
}

int CVICALLBACK windowCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int pfPanelHandle;
        GetPanelHandleFromTabPage(PANEL, PANEL_TABCONTROL, 1, &pfPanelHandle);
        SetCtrlVal(pfPanelHandle, TAB_PF_RBUTTON_RECTANGULAR, 0);
        SetCtrlVal(pfPanelHandle, TAB_PF_RBUTTON_HAMMING, 0);
        SetCtrlVal(pfPanelHandle, TAB_PF_RBUTTON_BLACKMAN, 0);
        SetCtrlVal(pfPanelHandle, control, 1);
        lpfCallback(panel, control, event, callbackData, eventData1, eventData2);
    }
    return 0;
}

int CVICALLBACK lpfCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int pfPanelHandle, i, window = 0, length, cutoffValue;
        double cutoff, max = 0, fq = 0.1;
        double kernelData[10000], kernelFftData[10000], kernelImData[10000], signalXData[10000];
        GetPanelHandleFromTabPage(PANEL, PANEL_TABCONTROL, 1, &pfPanelHandle);
        GetCtrlVal(pfPanelHandle, TAB_PF_RBUTTON_RECTANGULAR, &window);
        if (window)
            window = WINDOW_RECTANGULAR;
        GetCtrlVal(pfPanelHandle, TAB_PF_RBUTTON_HAMMING, &window);
        if (window)
            window = WINDOW_HAMMING;
        GetCtrlVal(pfPanelHandle, TAB_PF_RBUTTON_BLACKMAN, &window);
        if (window)
            window = WINDOW_BLACKMAN;
        GetCtrlVal(pfPanelHandle, TAB_PF_NUMERIC_LPF_LENGTH, &length);
        GetCtrlVal(pfPanelHandle, TAB_PF_NUMERIC_LPF_CUTOFF, &cutoffValue);
        cutoff = (double)cutoffValue / length;
        calcLpfKernel(window, length, cutoff, kernelData);
        GetCtrlVal(pfPanelHandle, TAB_PF_RBUTTON_HPF, &window);
        if (window)
            spectralInversion(kernelData, length);
        for (i = 0; i < length; i++)
            signalXData[i] = (double)i * fq / length;
        for (i = 0; i < length; i++)
            if (max < kernelData[i])
                max = kernelData[i];
        for (i = 0; i < length; i++)
            kernelData[i] /= max;
        DeleteGraphPlot(pfPanelHandle, TAB_PF_GRAPH_IMPULSE, -1, VAL_DELAYED_DRAW);
        SetAxisScalingMode(pfPanelHandle, TAB_PF_GRAPH_IMPULSE, VAL_LEFT_YAXIS, VAL_MANUAL, 0, 1);
        SetAxisScalingMode(pfPanelHandle, TAB_PF_GRAPH_IMPULSE, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0, fq);
        PlotXY(pfPanelHandle, TAB_PF_GRAPH_IMPULSE, signalXData, kernelData, length, VAL_DOUBLE, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED);
        for (i = 0; i < length; i++)
            kernelImData[i] = 0;
        GetCtrlVal(pfPanelHandle, TAB_PF_RBUTTON_HPF, &window);
        if (!window)
            calcLpfKernel(window, length, cutoff, kernelData);
        FFT(kernelData, kernelImData, length);
        for (i = 0; i < length; i++)
            kernelFftData[i] = fabs(sqrt(pow(kernelData[i], 2) + pow(kernelImData[i], 2)));
        for (i = 0; i < length; i++)
            signalXData[i] = i;
        for (i = 0; i < length; i++)
            if (max < kernelFftData[i])
                max = kernelFftData[i];
        for (i = 0; i < length; i++)
            kernelFftData[i] /= max;
        DeleteGraphPlot(pfPanelHandle, TAB_PF_GRAPH_FREQUENCY, -1, VAL_DELAYED_DRAW);
        SetAxisScalingMode(pfPanelHandle, TAB_PF_GRAPH_FREQUENCY, VAL_LEFT_YAXIS, VAL_MANUAL, 0, 1);
        SetAxisScalingMode(pfPanelHandle, TAB_PF_GRAPH_FREQUENCY, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0.0, length / 2);
        PlotXY(pfPanelHandle, TAB_PF_GRAPH_FREQUENCY, signalXData, kernelFftData, length, VAL_DOUBLE, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED);
        FILE* pFile = fopen("kernel_FFT.txt", "w");
        if (pFile)
        {
            for (i = 0; i < length; i++)
                fprintf(pFile, "%i\t%lf\n", i, kernelFftData[i]);
            fclose(pFile);
        }
    }
    return 0;
}

int CVICALLBACK ai4PfCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int ai4PanelHandle;
        GetPanelHandleFromTabPage(PANEL, PANEL_TABCONTROL, 2, &ai4PanelHandle);
        SetCtrlVal(ai4PanelHandle, TAB_AI4_RBUTTON_LPF, 0);
        SetCtrlVal(ai4PanelHandle, TAB_AI4_RBUTTON_HPF, 0);
        SetCtrlVal(ai4PanelHandle, control, 1);
    }
    return 0;
}

int CVICALLBACK ai4WindowCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int ai4PanelHandle;
        GetPanelHandleFromTabPage(PANEL, PANEL_TABCONTROL, 2, &ai4PanelHandle);
        SetCtrlVal(ai4PanelHandle, TAB_AI4_RBUTTON_RECTANGULAR, 0);
        SetCtrlVal(ai4PanelHandle, TAB_AI4_RBUTTON_HAMMING, 0);
        SetCtrlVal(ai4PanelHandle, TAB_AI4_RBUTTON_BLACKMAN, 0);
        SetCtrlVal(ai4PanelHandle, control, 1);
    }
    return 0;
}

int CVICALLBACK timerAi4Callback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    static double kernelData[NUM_SAMPLES], kernelReData[NUM_SAMPLES], kernelImData[NUM_SAMPLES], kernelFftData[NUM_SAMPLES], kernelFftGraphData[NUM_SAMPLES];
    if (event == EVENT_TIMER_TICK)
    {
        int ai4PanelHandle, i, n, window = 0, length, cutoffValue;
        double cutoff, max = 0, fq = 0.1;
        double signalData[NUM_SAMPLES], signalXData[NUM_SAMPLES], signalFftData[NUM_SAMPLES], signalFilteredData[NUM_SAMPLES + 40001], signalReData[NUM_SAMPLES], signalImData[NUM_SAMPLES];
        int activeTab;
        GetActiveTabPage(PANEL, PANEL_TABCONTROL, &activeTab);
        if (activeTab != 2)
            return 0;
        GetPanelHandleFromTabPage(PANEL, PANEL_TABCONTROL, 2, &ai4PanelHandle);
        GetCtrlVal(ai4PanelHandle, TAB_AI4_RBUTTON_RECTANGULAR, &window);
        if (window)
            window = WINDOW_RECTANGULAR;
        GetCtrlVal(ai4PanelHandle, TAB_AI4_RBUTTON_HAMMING, &window);
        if (window)
            window = WINDOW_HAMMING;
        GetCtrlVal(ai4PanelHandle, TAB_AI4_RBUTTON_BLACKMAN, &window);
        if (window)
            window = WINDOW_BLACKMAN;
        GetCtrlVal(ai4PanelHandle, TAB_AI4_NUMERIC_LPF_LENGTH, &length);
        GetCtrlVal(ai4PanelHandle, TAB_AI4_NUMERIC_LPF_CUTOFF, &cutoffValue);
        cutoff = CUTOFF_SCALE * (double)cutoffValue / length;
        calcLpfKernel(window, length, cutoff, kernelData);
        GetCtrlVal(ai4PanelHandle, TAB_AI4_RBUTTON_HPF, &window);
        if (window)
            spectralInversion(kernelData, length);
        analogInClk(NULL, SAMPLING_RATE);
        analogInTrigger("APFI0", TRIGGER_LEVEL);
        analogRead(0, signalData, NUM_SAMPLES, &n);
        DeleteGraphPlot(ai4PanelHandle, TAB_AI4_GRAPH_SIGNAL, -1, VAL_IMMEDIATE_DRAW);
        SetAxisScalingMode(ai4PanelHandle, TAB_AI4_GRAPH_SIGNAL, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0, n);
        PlotY(ai4PanelHandle, TAB_AI4_GRAPH_SIGNAL, signalData, n, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_YELLOW);
        for (i = 0; i < n; i++)
        {
            signalReData[i] = signalData[i];
            signalImData[i] = 0;
        }
        FFT(signalReData, signalImData, NUM_SAMPLES);
        max = 0;
        for (i = 0; i < n; i++)
        {
            signalFftData[i] = sqrt(pow(signalReData[i], 2) + pow(signalImData[i], 2));
            if (max < signalFftData[i])
                max = signalFftData[i];
            signalXData[i] = i * FREQUENCY_SCALE;
        }
        for (i = 0; i < n; i++)
            signalFftData[i] /= max;
        for (i = 0; i < length; i++)
        {
            kernelReData[i] = kernelData[i];
            kernelImData[i] = 0;
        }
        FFT(kernelReData, kernelImData, length);
        max = 0;
        for (i = 0; i < length; i++)
        {
            kernelFftData[i] = sqrt(pow(kernelReData[i], 2) + pow(kernelImData[i], 2));
            if (max < kernelFftData[i])
                max = kernelFftData[i];
        }
        for (i = 0; i < length; i++)
            kernelFftGraphData[i] = kernelFftData[i] / max;
        DeleteGraphPlot(ai4PanelHandle, TAB_AI4_GRAPH_FFT, -1, VAL_DELAYED_DRAW);
        DeleteGraphPlot(ai4PanelHandle, TAB_AI4_GRAPH_KERNEL_FFT, -1, VAL_DELAYED_DRAW);
        PlotXY(ai4PanelHandle, TAB_AI4_GRAPH_FFT, signalXData, signalFftData, n, VAL_DOUBLE, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED);
        SetAxisScalingMode(ai4PanelHandle, TAB_AI4_GRAPH_KERNEL_FFT, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0, 2 * cutoffValue * CUTOFF_SCALE);
        PlotY(ai4PanelHandle, TAB_AI4_GRAPH_KERNEL_FFT, kernelFftGraphData, length, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_RED);
        int s = n + length - 1;
        ConvolveEx(signalData, n, kernelData, length, ALGORITHM_CONCOR_FREQ_DOMAIN, signalFilteredData);
        DeleteGraphPlot(ai4PanelHandle, TAB_AI4_GRAPH_FILTERED, -1, VAL_IMMEDIATE_DRAW);
        SetAxisScalingMode(ai4PanelHandle, TAB_AI4_GRAPH_FILTERED, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0, s);
        PlotY(ai4PanelHandle, TAB_AI4_GRAPH_FILTERED, signalFilteredData, s, VAL_DOUBLE, VAL_THIN_LINE, VAL_NO_POINT, VAL_SOLID, 1, VAL_YELLOW);
    }
    return 0;
}