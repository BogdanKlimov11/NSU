#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#ifdef __cplusplus
    extern "C" {
#endif

#include "cvidef.h"

#define I2C_DELAY 0.001

int restrictIntValue(int* inputValue, int minValue, int maxValue);
void initI2c(void);
void writeI2c(int sda, int scl);
void readI2c(int* sda, int* scl);
void startI2c(void);
void stopI2c(void);
int getSda(void);
void sendBit(int bitValue);
void sendByte(int byteValue);
int receiveAck(void);
void sendAck(void);
void sendNack(void);
int receiveBit(void);
void receiveByte(int* byteValue);
int writeWord(int address, int subAddress, int wordValue);
int readWord(int address, int subAddress, int* outputData, int byteCount);
int readTemperature(double* temperatureValue);

#ifdef __cplusplus
    }
#endif

#endif