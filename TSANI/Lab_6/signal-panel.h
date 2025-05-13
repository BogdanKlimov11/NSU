#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define PANEL 1
#define PANEL_SIGNAL 2
#define PANEL_AMPLITUDE 3
#define PANEL_FREQUENCY 4
#define PANEL_FGEN_SLOT 5
#define PANEL_SCOPE_SLOT 6
#define PANEL_WAVEFORM_FORM 7
#define PANEL_IMPULSE_GENERATE 8
#define PANEL_GENERATE 9
#define PANEL_SAMPLE_RATE 10
#define PANEL_NUM_SAMPLES 11
#define PANEL_CHANNEL 12
#define PANEL_SLOT 13
#define PANEL_FOURIER 14
#define PANEL_FREQ_MAX 15
#define PANEL_FREQ_MIN 16
#define PANEL_STEP_DURATION 17
#define PANEL_FREQ_STEPS 18
#define PANEL_TIMER 19

int CVICALLBACK changeCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK generateCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK impulseGenerateCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK panelCallback(int panel, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK plotTimeCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK slotCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);

#ifdef __cplusplus
    }
#endif