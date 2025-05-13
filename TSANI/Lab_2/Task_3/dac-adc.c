#include <utility.h>
#include <tsani.h>

#include "avalon-interface.h"
#include "dac-adc.h"

#define BOARD_ADDRESS 2
#define DAC_CONTROL_REG 0x00
#define ADC_CONTROL_REG 0x10
#define CONTROL_VALUE 0x07
#define VOUT1_SUBADDRESS 0x02
#define VOUT2_SUBADDRESS 0x03
#define VIN1_SUBADDRESS 0x16
#define VIN2_SUBADDRESS 0x17
#define START_ADDRESS_REG 0x12
#define END_ADDRESS_REG 0x13
#define TIMER_REG 0x14
#define COMMAND_REG 0x11
#define COMMAND_START_IACK 0x03
#define SETI_BIT 0x04
#define IACK_BIT 0x02
#define INT3_BIT 0x20

int initDac(void)
{
    writeAvalon(BOARD_ADDRESS, DAC_CONTROL_REG, CONTROL_VALUE);
    return 0;
}

int initAdc(void)
{
    writeAvalon(BOARD_ADDRESS, ADC_CONTROL_REG, CONTROL_VALUE);
    return 0;
}

int outputDacAdc(int dacNum, double outputValue)
{
    restrictDoubleValue(&outputValue, 0, 3.3);
    int outputCode = (int)(outputValue * 255 / 3.3);
    outputDacAdcCode(dacNum, outputCode);
    return 0;
}

int outputDacAdcCode(int dacNum, int outputCode)
{
    int subAddress;
    if (dacNum == 1)
    {
        subAddress = VOUT1_SUBADDRESS;
    }
    else if (dacNum == 2)
    {
        subAddress = VOUT2_SUBADDRESS;
    }
    restrictIntValue(&outputCode, 0, 255);
    writeAvalon(BOARD_ADDRESS, subAddress, outputCode);
    return 0;
}

int inputDacAdc(int adcNum, double* inputValue)
{
    int statusValue = 0x03;
    int subAddress;
    if (adcNum == 1)
    {
        subAddress = VIN1_SUBADDRESS;
    }
    else if (adcNum == 2)
    {
        subAddress = VIN2_SUBADDRESS;
    }
    writeAvalon(BOARD_ADDRESS, START_ADDRESS_REG, 0x00);
    writeAvalon(BOARD_ADDRESS, END_ADDRESS_REG, 0x00);
    writeAvalon(BOARD_ADDRESS, TIMER_REG, 0x00);
    writeAvalon(BOARD_ADDRESS, COMMAND_REG, COMMAND_START_IACK);
    while ((statusValue & 0x01) == 0x01)
    {
        Delay(0.1);
        readAvalon(BOARD_ADDRESS, COMMAND_REG, &statusValue);
    }
    readAvalon(BOARD_ADDRESS, subAddress, &statusValue);
    *inputValue = (double)(statusValue * (2.56 / 1024));
    return 0;
}

int setInterrupt(void)
{
    int statusValue;
    readAvalon(BOARD_ADDRESS, COMMAND_REG, &statusValue);
    writeAvalon(BOARD_ADDRESS, COMMAND_REG, (statusValue | SETI_BIT));
    return 0;
}

int acknowledgeInterrupt(void)
{
    int statusValue;
    readAvalon(BOARD_ADDRESS, COMMAND_REG, &statusValue);
    writeAvalon(BOARD_ADDRESS, COMMAND_REG, (statusValue | IACK_BIT));
    return 0;
}

int inputDacAdcIack(int adcNum, double* inputValue)
{
    unsigned char interruptStatus = 0;
    int subAddress, statusValue;
    if (adcNum == 1)
    {
        subAddress = VIN1_SUBADDRESS;
    }
    else if (adcNum == 2)
    {
        subAddress = VIN2_SUBADDRESS;
    }
    writeAvalon(BOARD_ADDRESS, START_ADDRESS_REG, 0x00);
    writeAvalon(BOARD_ADDRESS, END_ADDRESS_REG, 0x00);
    writeAvalon(BOARD_ADDRESS, TIMER_REG, 0x00);
    writeAvalon(BOARD_ADDRESS, COMMAND_REG, COMMAND_START_IACK);
    while ((interruptStatus & INT3_BIT) != INT3_BIT)
    {
        Delay(0.1);
        portIn(2, &interruptStatus);
    }
    readAvalon(BOARD_ADDRESS, subAddress, &statusValue);
    *inputValue = (double)(statusValue * (2.56 / 1024));
    acknowledgeInterrupt();
    return 0;
}