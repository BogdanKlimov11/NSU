#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define PANEL 1
#define PANEL_LED_OVERHEAT 2
#define PANEL_BUTTON_PID_CONTROL 3
#define PANEL_BUTTON_MANUAL_HEAT 4
#define PANEL_NUMERIC_CURRENT_TEMP 5
#define PANEL_SPLITTER_2 6
#define PANEL_SPLITTER 7
#define PANEL_NUMERIC_DESIRED_TEMP 8
#define PANEL_TIMER_STATE_MONITOR 9
#define PANEL_NUMERIC_POWER 10
#define PANEL_TIMER_MAX_POWER 11
#define PANEL_TIMER_PID_CONTROL 12
#define PANEL_TIMER_HEAT 13
#define PANEL_STRIPCHART_TEMP 14
#define PANEL_NUMERIC_KD 15
#define PANEL_NUMERIC_KI 16
#define PANEL_NUMERIC_ERROR 17
#define PANEL_NUMERIC_DERIVATIVE 18
#define PANEL_NUMERIC_INTEGRAL 19
#define PANEL_NUMERIC_KP 20
#define PANEL_TIMER_ON_LOAD 21

int CVICALLBACK pidControlButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK heatButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK panelCallback(int panel, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK pidControlCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK heatCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK maxPowerCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK onLoadCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK stateMonitorCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);

#ifdef __cplusplus
    }
#endif