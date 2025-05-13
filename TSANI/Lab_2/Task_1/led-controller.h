#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define PANEL 1
#define PANEL_LED_0 2
#define PANEL_LED_1 3
#define PANEL_LED_2 4
#define PANEL_LED_3 5
#define PANEL_LED_4 6
#define PANEL_LED_5 7
#define PANEL_LED_6 8
#define PANEL_LED_7 9
#define PANEL_BUTTON_7 10
#define PANEL_BUTTON_6 11
#define PANEL_BUTTON_5 12
#define PANEL_BUTTON_4 13
#define PANEL_BUTTON_3 14
#define PANEL_BUTTON_2 15
#define PANEL_BUTTON_1 16
#define PANEL_BUTTON_0 17
#define PANEL_TIMER 18
#define PANEL_TOGGLE_BUTTON 19

int CVICALLBACK ledButtonCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK panelCallback(int panel, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK toggleButtonCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK runningLightCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);

#ifdef __cplusplus
    }
#endif