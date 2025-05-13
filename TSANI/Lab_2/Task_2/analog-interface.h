#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define PANEL 1
#define PANEL_NUMERIC_SLIDE 2
#define PANEL_NUMERIC_METER 3
#define PANEL_COMMAND_BUTTON 4

int CVICALLBACK measureCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK panelCallback(int panel, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK slideCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);

#ifdef __cplusplus
    }
#endif