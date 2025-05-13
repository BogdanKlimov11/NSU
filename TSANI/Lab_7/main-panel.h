#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define PANEL 1
#define PANEL_RADIOBUTTON_SINE 2
#define PANEL_RADIOBUTTON_SQUARE 3
#define PANEL_RADIOBUTTON_TRIANGLE 4
#define PANEL_NUMERIC_DC_OFFSET 5
#define PANEL_NUMERIC_DEVIATION 6
#define PANEL_NUMERIC_AMPLITUDE 7
#define PANEL_NUMERIC_MOD_FREQUENCY 8
#define PANEL_NUMERIC_FREQUENCY 9
#define PANEL_STRING_2 10
#define PANEL_STRING_STATE 11
#define PANEL_TBUTTON_INIT 12
#define PANEL_TEXTMSG_STEP1 13
#define PANEL_TEXTMSG_STEP3 14
#define PANEL_TEXTMSG_STEP2 15
#define PANEL_TBUTTON_MODULATION 16
#define PANEL_TBUTTON_GENERATOR 17
#define PANEL_TEXTMSG_WAVEFORM 18
#define PANEL_TEXTMSG_DC_OFFSET 19
#define PANEL_TEXTMSG_AMPLITUDE 20
#define PANEL_TEXTMSG_3 21
#define PANEL_TEXTMSG_DEVIATION 22
#define PANEL_TEXTMSG_FREQUENCY 23
#define PANEL_TEXTMSG_MOD_FREQUENCY 24
#define PANEL_TEXTMSG_2 25
#define PANEL_NUMERIC_DEBUG 26
#define PANEL_TABCONTROL 27

#define TAB_AI0_GRAPH_SIGNAL 2
#define TAB_AI0_GRAPH_FFT 3
#define TAB_AI0_TIMER 4

#define TAB_AI4_GRAPH_KERNEL_FFT 2
#define TAB_AI4_GRAPH_FFT 3
#define TAB_AI4_GRAPH_SIGNAL 4
#define TAB_AI4_GRAPH_FILTERED 5
#define TAB_AI4_RBUTTON_HPF 6
#define TAB_AI4_RBUTTON_LPF 7
#define TAB_AI4_RBUTTON_BLACKMAN 8
#define TAB_AI4_TEXTMSG_WAVEFORM_2 9
#define TAB_AI4_RBUTTON_HAMMING 10
#define TAB_AI4_RBUTTON_RECTANGULAR 11
#define TAB_AI4_TEXTMSG_STEP2 12
#define TAB_AI4_TEXTMSG_WAVEFORM 13
#define TAB_AI4_NUMERIC_LPF_CUTOFF 14
#define TAB_AI4_NUMERIC_LPF_LENGTH 15
#define TAB_AI4_TEXTMSG_FREQUENCY 16
#define TAB_AI4_TEXTMSG_FREQUENCY_2 17
#define TAB_AI4_TIMER_AI4 18

#define TAB_PF_GRAPH_FREQUENCY 2
#define TAB_PF_GRAPH_IMPULSE 3
#define TAB_PF_RBUTTON_HPF 4
#define TAB_PF_RBUTTON_LPF 5
#define TAB_PF_RBUTTON_BLACKMAN 6
#define TAB_PF_TEXTMSG_WAVEFORM_2 7
#define TAB_PF_RBUTTON_HAMMING 8
#define TAB_PF_RBUTTON_RECTANGULAR 9
#define TAB_PF_TEXTMSG_STEP2 10
#define TAB_PF_TEXTMSG_WAVEFORM 11
#define TAB_PF_NUMERIC_LPF_LENGTH 12
#define TAB_PF_TEXTMSG_FREQUENCY_2 13
#define TAB_PF_NUMERIC_LPF_CUTOFF 14
#define TAB_PF_TEXTMSG_FREQUENCY 15

#define MENUBAR 1
#define MENUBAR_MENU1 2
#define MENUBAR_2 2
#define MENUBAR_2_MENU1 2
#define MENUBAR_2_MENU2 3
#define MENUBAR_3 3
#define MENUBAR_3_MENU1 2
#define MENUBAR_3_MENU2 3

int CVICALLBACK lpfCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK amplitudeCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK dcOffsetCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK frequencyCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK deviationCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK modFrequencyCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK panelCallback(int panel, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK waveformSelectCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK ai4PfCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK ai4WindowCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK pfCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK windowCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK generatorCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK modulationCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK tbuttonInitCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK timerAi4Callback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK timerCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);

#ifdef __cplusplus
    }
#endif