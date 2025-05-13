#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define PANEL 1
#define PANEL_DATA_READ 2
#define PANEL_SUBADDRESS_READ 3
#define PANEL_ADDRESS_READ 4
#define PANEL_DATA_WRITE 5
#define PANEL_SUBADDRESS_WRITE 6
#define PANEL_ADDRESS_WRITE 7
#define PANEL_READ_BUTTON 8
#define PANEL_WRITE_BUTTON 9
#define PANEL_ADC2_METER 10
#define PANEL_DAC2_SLIDE 11
#define PANEL_ADC2_MEASURE_BUTTON 12
#define PANEL_TEXT_MSG_4 13
#define PANEL_ADC1_METER 14
#define PANEL_DAC1_SLIDE 15
#define PANEL_ADC1_MEASURE_BUTTON 16
#define PANEL_TEXT_MSG_2 17
#define PANEL_TEXT_MSG 18
#define PANEL_ADC_IACK_BUTTON 19
#define PANEL_ADC_SETI_BUTTON 20
#define PANEL_TEXT_MSG_3 21

int CVICALLBACK adc1MeasureCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK adc2MeasureCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK adcIackCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK adcSetInterruptCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK readButtonCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK writeButtonCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK dac1SlideCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK dac2SlideCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int CVICALLBACK panelCallback(int panel, int event, void *callbackData, int eventData1, int eventData2);

#ifdef __cplusplus
    }
#endif