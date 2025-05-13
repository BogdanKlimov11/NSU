#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define PANEL 1
#define PANEL_GRAPH_SPECTRUM 2
#define PANEL_GRAPH 3
#define PANEL_PHASE 4
#define PANEL_FREQUENCY 5
#define PANEL_AMPLITUDE 6
#define PANEL_NOISE 7
#define PANEL_BUTTON_SAVE 8
#define PANEL_TIMER 9

int CVICALLBACK editAmplitude(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK editFrequency(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK editNoise(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK editPhase(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK panelCallback(int panel, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK saveButtonCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK timerCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);

#ifdef __cplusplus
    }
#endif