#ifndef SCPI_INTERFACE_H
#define SCPI_INTERFACE_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "cvidef.h"

#define PORT_NUMBER 8
#define TIMEOUT 0.4
#define TERMINATION_CHAR "\n"
#define BUFFER_SIZE 64
#define COMMAND_DELAY 0.1

void checkValueInt(int* value, int min, int max);
void checkValueDouble(double* value, double min, double max);
int comInit(int portNumber, double timeout);
int comDeinit(int portNumber);
int comGeneratorOff(int portNumber);
int comWriteInt(const char command[], int param);
int comWriteDouble(const char command[], double param);
int comReadInt(char* buffer);
int comReadDouble(char* buffer);

#ifdef __cplusplus
    }
#endif

#endif