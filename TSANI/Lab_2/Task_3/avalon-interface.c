#include <utility.h>
#include <tsani.h>

#include "dac-adc.h"
#include "avalon-interface.h"

#define PORT_MASK_ALL 0xff
#define PORT_MASK_CONTROL 0x07
#define PORT_VALUE_ALE0_RW1 0x06
#define PORT_VALUE_ALE1_RW1 0x07
#define PORT_VALUE_ALE0_RW0 0x02
#define PORT_VALUE_ALE1_R0W0 0x04

int restrictIntValue(int* inputValue, int minValue, int maxValue)
{
    if (*inputValue > maxValue)
    {
        *inputValue = maxValue;
    }
    else if (*inputValue < minValue)
    {
        *inputValue = minValue;
    }
    return 0;
}

int restrictDoubleValue(double* inputValue, double minValue, double maxValue)
{
    if (*inputValue > maxValue)
    {
        *inputValue = maxValue;
    }
    else if (*inputValue < minValue)
    {
        *inputValue = minValue;
    }
    return 0;
}

int initAvalon(void)
{
    portMask(0, PORT_MASK_ALL);
    portMask(1, PORT_MASK_ALL);
    portMask(2, PORT_MASK_CONTROL);
    portOut(0, 0x00);
    portOut(1, 0x00);
    portOut(2, PORT_VALUE_ALE0_RW1);
    return 0;
}

int writeAvalon(int address, int subAddress, int inputValue)
{
    int fullAddress = address + (subAddress << 3);
    portMask(0, PORT_MASK_ALL);
    portMask(1, PORT_MASK_ALL);
    portMask(2, PORT_MASK_CONTROL);
    restrictIntValue(&fullAddress, 0, 65535);
    restrictIntValue(&inputValue, 0, 65535);
    portOut(0, (fullAddress & 0xff));
    portOut(1, ((fullAddress >> 8) & 0xff));
    portOut(2, PORT_VALUE_ALE1_RW1);
    portOut(2, PORT_VALUE_ALE0_RW1);
    portOut(0, (inputValue & 0xff));
    portOut(1, ((inputValue >> 8) & 0xff));
    portOut(2, PORT_VALUE_ALE0_RW0);
    portOut(2, PORT_VALUE_ALE0_RW1);
    return 0;
}

int readAvalon(int address, int subAddress, int* outputValue)
{
    int fullAddress = address + (subAddress << 3);
    unsigned char dataLow, dataHigh;
    portMask(0, PORT_MASK_ALL);
    portMask(1, PORT_MASK_ALL);
    portMask(2, PORT_MASK_CONTROL);
    restrictIntValue(&fullAddress, 0, 65535);
    portOut(0, (fullAddress & 0xff));
    portOut(1, ((fullAddress >> 8) & 0xff));
    portOut(2, PORT_VALUE_ALE1_RW1);
    portOut(2, PORT_VALUE_ALE0_RW1);
    portOut(2, PORT_VALUE_ALE1_R0W0);
    portMask(0, 0x00);
    portMask(1, 0x00);
    portIn(0, &dataLow);
    portIn(1, &dataHigh);
    portOut(2, PORT_VALUE_ALE0_RW1);
    portMask(0, PORT_MASK_ALL);
    portMask(1, PORT_MASK_ALL);
    *outputValue = (((int)dataHigh & 0xff) << 8) + ((int)dataLow & 0xff);
    return 0;
}