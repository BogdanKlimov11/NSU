#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define PANEL 1
#define PANEL_GRAPH 2
#define PANEL_SC 3
#define PANEL_GRAPH_2 4
#define PANEL_GRAPH_3 5
#define PANEL_NUMERIC_X_COORD 6
#define PANEL_NUMERIC_Y_COORD 7
#define PANEL_BUTTON_INIT 8
#define PANEL_BUTTON_SEPARATE 9
#define PANEL_BUTTON_OK 10
#define PANEL_TIMER_READ 11
#define PANEL_TIMER_PROCESS 12

int CVICALLBACK panelCallback(int panel, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK initButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK separateButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK okButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK readTimerCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK processTimerCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);

#ifdef __cplusplus
    }
#endif