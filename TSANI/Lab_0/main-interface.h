#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define PANEL 1
#define PANEL_NUMERIC_THERM 2
#define PANEL_LED 3
#define PANEL_COMMAND_BUTTON 4
#define PANEL_COLOR_NUM 5

int CVICALLBACK panelCallback(int panel, int event, void *callbackData, int eventData1, int eventData2);

#ifdef __cplusplus
    }
#endif