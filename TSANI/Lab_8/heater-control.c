#include <formatio.h>
#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <tsani.h>

#include "toolbox.h"
#include "heater-control.h"

#define PORT_HEAT 1
#define HEAT_LINE 7
#define OVERHEAT_LINE 6
#define ANALOG_INPUT_CHANNEL 2
#define TEMP_SCALE 100.0
#define MAX_POWER 100.0
#define MIN_POWER 0.0
#define KP_DEFAULT 10.0
#define KI_DEFAULT 1.0
#define KD_DEFAULT 1.0
#define NI6251_SLOT 2

static int panelHandle;

void restrictInt(int* value, int min, int max)
{
    if (*value > max)
        *value = max;
    else if (*value < min)
        *value = min;
}

void restrictDouble(double* value, double min, double max)
{
    if (*value > max)
        *value = max;
    else if (*value < min)
        *value = min;
}

void heatSet(int state)
{
    restrictInt(&state, 0, 1);
    portOut(PORT_HEAT, state << HEAT_LINE);
}

void readTemperature(double* value)
{
    double temp;
    analogIn(ANALOG_INPUT_CHANNEL, &temp);
    *value = temp * TEMP_SCALE;
}

void readOverheat(int* state)
{
    unsigned char overheat;
    portIn(PORT_HEAT, &overheat);
    overheat &= 1 << OVERHEAT_LINE;
    *state = (overheat == (1 << OVERHEAT_LINE)) ? 0 : 1;
}

int main(int argc, char* argv[])
{
    int error = 0;
    ni6251Slot(NI6251_SLOT);
    portMask(0, 0x00);
    portMask(PORT_HEAT, 0x80);
    portMask(2, 0x00);
    portOut(PORT_HEAT, 0x00);
    nullChk(InitCVIRTE(0, argv, 0));
    errChk(panelHandle = LoadPanel(0, "heater-control.uir", PANEL));
    errChk(DisplayPanel(panelHandle));
    errChk(RunUserInterface());

Error:
    DiscardPanel(panelHandle);
    ni6251Close();
    return 0;
}

int CVICALLBACK panelCallback(int panel, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_CLOSE)
        QuitUserInterface(0);
    return 0;
}

int CVICALLBACK onLoadCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_TIMER_TICK)
    {
        double timeStep;
        SetCtrlAttribute(PANEL, PANEL_TIMER_ON_LOAD, ATTR_ENABLED, 0);
        GetCtrlAttribute(PANEL, PANEL_TIMER_PID_CONTROL, ATTR_INTERVAL, &timeStep);
        SetCtrlAttribute(PANEL, PANEL_NUMERIC_KP, ATTR_DIMMED, 0);
        SetCtrlAttribute(PANEL, PANEL_NUMERIC_KI, ATTR_DIMMED, 0);
        SetCtrlAttribute(PANEL, PANEL_NUMERIC_KD, ATTR_DIMMED, 0);
    }
    return 0;
}

int CVICALLBACK stateMonitorCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_TIMER_TICK)
    {
        double temperature;
        int overheat;
        readTemperature(&temperature);
        readOverheat(&overheat);
        SetCtrlVal(panelHandle, PANEL_NUMERIC_CURRENT_TEMP, temperature);
        SetCtrlVal(panelHandle, PANEL_LED_OVERHEAT, overheat);
        PlotStripChartPoint(panelHandle, PANEL_STRIPCHART_TEMP, temperature);
    }
    return 0;
}

int CVICALLBACK heatButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int state;
        GetCtrlVal(PANEL, PANEL_BUTTON_MANUAL_HEAT, &state);
        SetCtrlAttribute(PANEL, PANEL_TIMER_HEAT, ATTR_ENABLED, state);
        SetCtrlAttribute(PANEL, PANEL_TIMER_MAX_POWER, ATTR_ENABLED, 0);
        SetCtrlAttribute(PANEL, PANEL_TIMER_PID_CONTROL, ATTR_ENABLED, 0);
        SetCtrlAttribute(PANEL, PANEL_BUTTON_PID_CONTROL, ATTR_DIMMED, state);
        SetCtrlAttribute(PANEL, PANEL_NUMERIC_DESIRED_TEMP, ATTR_CTRL_MODE, state ? VAL_INDICATOR : VAL_HOT);
    }
    return 0;
}

int CVICALLBACK heatCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_TIMER_TICK)
    {
        double power, interval;
        GetCtrlVal(PANEL, PANEL_NUMERIC_POWER, &power);
        GetCtrlAttribute(PANEL, PANEL_TIMER_HEAT, ATTR_INTERVAL, &interval);
        heatSet(1);
        Delay(interval * power / MAX_POWER);
        heatSet(0);
    }
    return 0;
}

int CVICALLBACK pidControlButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_COMMIT)
    {
        int state;
        double measuredTemperature, desiredTemperature;
        readTemperature(&measuredTemperature);
        GetCtrlVal(PANEL, PANEL_NUMERIC_DESIRED_TEMP, &desiredTemperature);
        GetCtrlVal(PANEL, PANEL_BUTTON_PID_CONTROL, &state);
        if (desiredTemperature > measuredTemperature && state)
        {
            SetCtrlAttribute(PANEL, PANEL_TIMER_HEAT, ATTR_ENABLED, 1);
            SetCtrlVal(PANEL, PANEL_NUMERIC_POWER, MAX_POWER);
        }
        else
        {
            SetCtrlAttribute(PANEL, PANEL_TIMER_HEAT, ATTR_ENABLED, 0);
            SetCtrlVal(PANEL, PANEL_NUMERIC_POWER, MIN_POWER);
        }
        SetCtrlAttribute(PANEL, PANEL_TIMER_MAX_POWER, ATTR_ENABLED, state);
        SetCtrlAttribute(PANEL, PANEL_TIMER_HEAT, ATTR_ENABLED, state);
        SetCtrlAttribute(PANEL, PANEL_BUTTON_MANUAL_HEAT, ATTR_DIMMED, state);
        SetCtrlAttribute(PANEL, PANEL_NUMERIC_POWER, ATTR_CTRL_MODE, state ? VAL_INDICATOR : VAL_HOT);
        SetCtrlAttribute(PANEL, PANEL_NUMERIC_DESIRED_TEMP, ATTR_CTRL_MODE, state ? VAL_INDICATOR : VAL_HOT);
        SetCtrlAttribute(PANEL, PANEL_TIMER_PID_CONTROL, ATTR_ENABLED, 0);
    }
    return 0;
}

int CVICALLBACK maxPowerCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_TIMER_TICK)
    {
        double measuredTemperature, desiredTemperature, prevTemperature;
        double integralTerm = 0.0;
        prevTemperature = measuredTemperature;
        readTemperature(&measuredTemperature);
        GetCtrlVal(PANEL, PANEL_NUMERIC_DESIRED_TEMP, &desiredTemperature);
        if ((desiredTemperature - prevTemperature) * (desiredTemperature - measuredTemperature) < 0)
        {
            heatSet(0);
            SetCtrlAttribute(PANEL, PANEL_TIMER_MAX_POWER, ATTR_ENABLED, 0);
            SetCtrlAttribute(PANEL, PANEL_TIMER_PID_CONTROL, ATTR_ENABLED, 1);
            SetCtrlAttribute(PANEL, PANEL_NUMERIC_DESIRED_TEMP, ATTR_CTRL_MODE, VAL_HOT);
        }
    }
    return 0;
}

int CVICALLBACK pidControlCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2)
{
    if (event == EVENT_TIMER_TICK)
    {
        double measuredTemperature, desiredTemperature, error, prevError, integralTerm, derivativeTerm, outputPower;
        double timeStep, kp, ki, kd;
        GetCtrlVal(PANEL, PANEL_NUMERIC_KP, &kp);
        GetCtrlVal(PANEL, PANEL_NUMERIC_KI, &ki);
        GetCtrlVal(PANEL, PANEL_NUMERIC_KD, &kd);
        prevError = error;
        readTemperature(&measuredTemperature);
        GetCtrlVal(PANEL, PANEL_NUMERIC_DESIRED_TEMP, &desiredTemperature);
        GetCtrlAttribute(PANEL, PANEL_TIMER_PID_CONTROL, ATTR_INTERVAL, &timeStep);
        error = desiredTemperature - measuredTemperature;
        integralTerm += error * timeStep;
        derivativeTerm = (error - prevError) / timeStep;
        outputPower = kp * error + ki * integralTerm + kd * derivativeTerm;
        restrictDouble(&outputPower, MIN_POWER, MAX_POWER);
        SetCtrlVal(panelHandle, PANEL_NUMERIC_POWER, outputPower);
        SetCtrlVal(panelHandle, PANEL_NUMERIC_KP, kp);
        SetCtrlVal(panelHandle, PANEL_NUMERIC_KI, ki);
        SetCtrlVal(panelHandle, PANEL_NUMERIC_KD, kd);
        SetCtrlVal(panelHandle, PANEL_NUMERIC_ERROR, error);
        SetCtrlVal(panelHandle, PANEL_NUMERIC_INTEGRAL, integralTerm);
        SetCtrlVal(panelHandle, PANEL_NUMERIC_DERIVATIVE, derivativeTerm);
    }
    return 0;
}