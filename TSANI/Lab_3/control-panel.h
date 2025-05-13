#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define PANEL 1
#define PANEL_TAB_CONTROL 2

#define TAB_ADC_ADC_BUTTON_2 2
#define TAB_ADC_ADC_BUTTON 3
#define TAB_ADC_DIFF 4
#define TAB_ADC_INTEG 5
#define TAB_ADC_GAIN 6
#define TAB_ADC_LSB 7
#define TAB_ADC_OFFSET 8
#define TAB_ADC_GRAPH_2 9
#define TAB_ADC_GRAPH 10

#define TAB_DAC_DAC_BUTTON_2 2
#define TAB_DAC_DAC_BUTTON 3
#define TAB_DAC_DIFF 4
#define TAB_DAC_INTEG 5
#define TAB_DAC_GAIN 6
#define TAB_DAC_LSB 7
#define TAB_DAC_OFFSET 8
#define TAB_DAC_STRIP_CHART 9
#define TAB_DAC_TIMER 10

#define TAB_DIFF_NUMBER 2
#define TAB_DIFF_NUMERIC_U 3
#define TAB_DIFF_HIST_BUTTON 4
#define TAB_DIFF_GRAPH 5

int CVICALLBACK adcButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK dacButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK dacLoadDataCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK histButtonCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK loadUserInterfaceCallback(int panel, int control, int event, void* callbackData, int eventData1, int eventData2);
int CVICALLBACK panelCallback(int panel, int event, void* callbackData, int eventData1, int eventData2);

#ifdef __cplusplus
    }
#endif