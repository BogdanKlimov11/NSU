#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define PANEL 1
#define PANEL_NUMERIC_TEMP 2
#define PANEL_BUTTON_SINGLE 3
#define PANEL_BUTTON_WORD 4
#define PANEL_STRING 5
#define PANEL_TEXT_MSG_3 6
#define PANEL_TEXT_MSG_2 7
#define PANEL_TEXT_MSG 8
#define PANEL_TBUTTON_LED 9
#define PANEL_TBUTTON_TEMPERATURE 10
#define PANEL_LED_7 11
#define PANEL_LED_6 12
#define PANEL_LED_5 13
#define PANEL_LED_4 14
#define PANEL_LED_3 15
#define PANEL_LED_2 16
#define PANEL_LED_1 17
#define PANEL_LED_0 18
#define PANEL_TIMER_TEMPERATURE 19
#define PANEL_NUMERIC_DEBUG_2 20
#define PANEL_NUMERIC_DEBUG 21
#define PANEL_TIMER_LED 22
#define PANEL_STRIP_CHART 23

int CVICALLBACK singleButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK wordButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK ledButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK panelCallback(int panel, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK ledTbuttonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK temperatureTbuttonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK ledTimerCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK temperatureTimerCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);

#ifdef __cplusplus
    }
#endif