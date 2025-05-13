#include <ansi_c.h>
#include <userint.h>
#include <rs232.h>

#include "toolbox.h"
#include "main-panel.h"
#include "scpi-interface.h"

void checkValueInt(int* value, int min, int max)
{
    if (*value > max)
        *value = max;
    else if (*value < min)
        *value = min;
}

void checkValueDouble(double* value, double min, double max)
{
    if (*value > max)
        *value = max;
    else if (*value < min)
        *value = min;
}

int comInit(int portNumber, double timeout)
{
    char portName[5] = {0};
    checkValueInt(&portNumber, 1, 9);
    sprintf(portName, "COM%i", portNumber);
    SetBreakOnLibraryErrors(0);
    if (OpenComConfig(portNumber, portName, 19200, 0, 8, 1, 0, 0) < 0)
    {
        SetCtrlVal(PANEL, PANEL_STRING_STATE, GetRS232ErrorString(ReturnRS232Err()));
        return -1;
    }
    if (SetComTime(portNumber, timeout) < 0)
    {
        char error[64] = {0};
        sprintf(error, "Error: 0x%X", GetComStat(portNumber));
        SetCtrlVal(PANEL, PANEL_STRING_STATE, error);
        return -1;
    }
    return 0;
}

int comDeinit(int portNumber)
{
    SetBreakOnLibraryErrors(0);
    if (GetComStat(portNumber) >= 0)
    {
        comGeneratorOff(portNumber);
        CloseCom(portNumber);
    }
    else
    {
        char error[64] = {0};
        sprintf(error, "Error: 0x%X", GetComStat(portNumber));
        SetCtrlVal(PANEL, PANEL_STRING_STATE, error);
        return -1;
    }
    return 0;
}

int comGeneratorOff(int portNumber)
{
    SetBreakOnLibraryErrors(0);
    return 0;
}

int comWriteInt(const char command[], int param)
{
    SetBreakOnLibraryErrors(0);
    char buffer[1024] = {0};
    sprintf(buffer, "%s %d%s", command, param, TERMINATION_CHAR);
    if (ComWrt(PORT_NUMBER, buffer, strlen(buffer)) != strlen(buffer))
        return ReturnRS232Err();
    Delay(COMMAND_DELAY);
    return 0;
}

int comWriteDouble(const char command[], double param)
{
    SetBreakOnLibraryErrors(0);
    char buffer[1024] = {0};
    sprintf(buffer, "%s %lf%s", command, param, TERMINATION_CHAR);
    if (ComWrt(PORT_NUMBER, buffer, strlen(buffer)) != strlen(buffer))
        return ReturnRS232Err();
    Delay(COMMAND_DELAY);
    return 0;
}

int comReadInt(char* buffer)
{
    SetBreakOnLibraryErrors(0);
    if (ComRdTerm(PORT_NUMBER, buffer, BUFFER_SIZE, (int)TERMINATION_CHAR[0]) <= 0)
        return ReturnRS232Err();
    Delay(COMMAND_DELAY);
    return 0;
}

int comReadDouble(char* buffer)
{
    SetBreakOnLibraryErrors(0);
    if (ComRdTerm(PORT_NUMBER, buffer, BUFFER_SIZE, (int)TERMINATION_CHAR[0]) <= 0)
        return ReturnRS232Err();
    Delay(COMMAND_DELAY);
    return 0;
}