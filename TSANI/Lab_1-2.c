#include <analysis.h>
#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <toolbox.h>

static int panelHandle;

double signal[1000];
double spectrum[1000];
double power[1000];
#define SAMPLE_RATE 1000.0

void CreateSignal(double amplitude, double frequency, double phase, double noiseLevel) {
    double timeStep = 1.0 / SAMPLE_RATE;
    for (int i = 0; i < 1000; i++) {
        signal[i] = amplitude * sin(2 * PI * frequency * i * timeStep + phase);
        signal[i] += Random(-noiseLevel, noiseLevel);
    }
}

void CreateSpectrum() {
    double real[1000], imag[1000] = {0};
    for (int i = 0; i < 1000; i++) {
        real[i] = signal[i];
    }
    FFT(real, imag, 1000);
    for (int i = 0; i < 500; i++) {
        spectrum[i] = sqrt(real[i] * real[i] + imag[i] * imag[i]);
    }
}

void CalculatePower() {
    for (int i = 0; i < 1000; i++) {
        power[i] = signal[i] * signal[i];
    }
}

void DrawGraphSignal() {
    DeleteGraphPlot(panelHandle, PANEL_Graf1, -1, VAL_IMMEDIATE_DRAW);
    SetAxisScalingMode(panelHandle, PANEL_Graf1, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0.0, 1000.0);
    SetAxisScalingMode(panelHandle, PANEL_Graf1, VAL_LEFT_YAXIS, VAL_MANUAL, -15.0, 15.0);
    PlotY(panelHandle, PANEL_Graf1, signal, 1000, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_GREEN);
}

void DrawGraphSpectrum() {
    DeleteGraphPlot(panelHandle, PANEL_Graf2, -1, VAL_IMMEDIATE_DRAW);
    SetAxisScalingMode(panelHandle, PANEL_Graf2, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0.0, 500.0);
    SetAxisScalingMode(panelHandle, PANEL_Graf2, VAL_LEFT_YAXIS, VAL_AUTOSCALE, 0.0, 0.0);
    PlotY(panelHandle, PANEL_Graf2, spectrum, 500, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_BLUE);
}

void DrawGraphPower() {
    DeleteGraphPlot(panelHandle, PANEL_Graf3, -1, VAL_IMMEDIATE_DRAW);
    SetAxisScalingMode(panelHandle, PANEL_Graf3, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0.0, 1000.0);
    SetAxisScalingMode(panelHandle, PANEL_Graf3, VAL_LEFT_YAXIS, VAL_MANUAL, 0.0, 225.0);
    PlotY(panelHandle, PANEL_Graf3, power, 1000, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_RED);
}

void SaveSignal() {
    FILE *file = fopen("signal.txt", "w");
    if (file == NULL) {
        MessagePopup("Error", "Cannot open file for saving");
        return;
    }
    for (int i = 0; i < 1000; i++) {
        fprintf(file, "%d\t%f\n", i, signal[i]);
    }
    fclose(file);
    MessagePopup("Success", "Signal saved to signal.txt");
}

int main(int argc, char *argv[]) {
    if (InitCVIRTE(0, argv, 0) == 0)
        return -1;
    if ((panelHandle = LoadPanel(0, "Sinus.uir", PANEL)) < 0)
        return -1;
    CreateSignal(10.0, 1.0, 0.0, 0.0);
    DrawGraphSignal();
    CreateSpectrum();
    DrawGraphSpectrum();
    CalculatePower();
    DrawGraphPower();
    DisplayPanel(panelHandle);
    RunUserInterface();
    DiscardPanel(panelHandle);
    return 0;
}

int CVICALLBACK panelCB(int panel, int event, void *callbackData, int eventData1, int eventData2) {
    if (event == EVENT_CLOSE)
        QuitUserInterface(0);
    return 0;
}

int CVICALLBACK Timer1(int panel, int control, int event, void *callbackData, int eventData1, int eventData2) {
    if (event == EVENT_TIMER_TICK) {
        double amplitude, frequency, phase, noiseLevel;
        GetCtrlVal(panelHandle, PANEL_Ampl, &amplitude);
        GetCtrlVal(panelHandle, PANEL_Freq, &frequency);
        GetCtrlVal(panelHandle, PANEL_Phase, &phase);
        GetCtrlVal(panelHandle, PANEL_Noise, &noiseLevel);
        CreateSignal(amplitude, frequency, phase, noiseLevel);
        DrawGraphSignal();
        CreateSpectrum();
        DrawGraphSpectrum();
        CalculatePower();
        DrawGraphPower();
    }
    return 0;
}

int CVICALLBACK Save1(int panel, int control, int event, void *callbackData, int eventData1, int eventData2) {
    if (event == EVENT_COMMIT) {
        SaveSignal();
    }
    return 0;
}

int CVICALLBACK Ampl(int panel, int control, int event, void *callbackData, int eventData1, int eventData2) {
    if (event == EVENT_COMMIT) {
        double amplitude, frequency, phase, noiseLevel;
        GetCtrlVal(panelHandle, PANEL_Ampl, &amplitude);
        GetCtrlVal(panelHandle, PANEL_Freq, &frequency);
        GetCtrlVal(panelHandle, PANEL_Phase, &phase);
        GetCtrlVal(panelHandle, PANEL_Noise, &noiseLevel);
        CreateSignal(amplitude, frequency, phase, noiseLevel);
        DrawGraphSignal();
        CreateSpectrum();
        DrawGraphSpectrum();
        CalculatePower();
        DrawGraphPower();
    }
    return 0;
}

int CVICALLBACK Freq(int panel, int control, int event, void *callbackData, int eventData1, int eventData2) {
    if (event == EVENT_COMMIT) {
        double amplitude, frequency, phase, noiseLevel;
        GetCtrlVal(panelHandle, PANEL_Ampl, &amplitude);
        GetCtrlVal(panelHandle, PANEL_Freq, &frequency);
        GetCtrlVal(panelHandle, PANEL_Phase, &phase);
        GetCtrlVal(panelHandle, PANEL_Noise, &noiseLevel);
        CreateSignal(amplitude, frequency, phase, noiseLevel);
        DrawGraphSignal();
        CreateSpectrum();
        DrawGraphSpectrum();
        CalculatePower();
        DrawGraphPower();
    }
    return 0;
}

int CVICALLBACK Phase(int panel, int control, int event, void *callbackData, int eventData1, int eventData2) {
    if (event == EVENT_COMMIT) {
        double amplitude, frequency, phase, noiseLevel;
        GetCtrlVal(panelHandle, PANEL_Ampl, &amplitude);
        GetCtrlVal(panelHandle, PANEL_Freq, &frequency);
        GetCtrlVal(panelHandle, PANEL_Phase, &phase);
        GetCtrlVal(panelHandle, PANEL_Noise, &noiseLevel);
        CreateSignal(amplitude, frequency, phase, noiseLevel);
        DrawGraphSignal();
        CreateSpectrum();
        DrawGraphSpectrum();
        CalculatePower();
        DrawGraphPower();
    }
    return 0;
}

int CVICALLBACK Noise(int panel, int control, int event, void *callbackData, int eventData1, int eventData2) {
    if (event == EVENT_COMMIT) {
        double amplitude, frequency, phase, noiseLevel;
        GetCtrlVal(panelHandle, PANEL_Ampl, &amplitude);
        GetCtrlVal(panelHandle, PANEL_Freq, &frequency);
        GetCtrlVal(panelHandle, PANEL_Phase, &phase);
        GetCtrlVal(panelHandle, PANEL_Noise, &noiseLevel);
        CreateSignal(amplitude, frequency, phase, noiseLevel);
        DrawGraphSignal();
        CreateSpectrum();
        DrawGraphSpectrum();
        CalculatePower();
        DrawGraphPower();
    }
    return 0;
}

int CVICALLBACK Graf1(int panel, int control, int event, void *callbackData, int eventData1, int eventData2) {
    switch (event) {
        case EVENT_COMMIT:
        case EVENT_LEFT_CLICK:
        case EVENT_RIGHT_CLICK:
        case EVENT_GOT_FOCUS:
        case EVENT_DISCARD:
            break;
    }
    return 0;
}

int CVICALLBACK Graf2(int panel, int control, int event, void *callbackData, int eventData1, int eventData2) {
    switch (event) {
        case EVENT_COMMIT:
        case EVENT_LEFT_CLICK:
        case EVENT_RIGHT_CLICK:
        case EVENT_GOT_FOCUS:
        case EVENT_DISCARD:
            break;
    }
    return 0;
}
