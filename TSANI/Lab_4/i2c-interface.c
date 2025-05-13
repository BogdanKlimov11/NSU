#include <utility.h>
#include <tsani.h>

#include "i2c-interface.h"

#define PORT0_MASK 0x7f
#define PORT1_MASK 0x00
#define I2C1_SDA_OUT_SHIFT 3
#define I2C1_SCL_OUT_SHIFT 4
#define I2C1_SDA_IN_SHIFT 7
#define I2C1_SCL_IN_MASK 0x01
#define I2C2_SDA_OUT_BIT (1 << 5)
#define I2C2_SCL_OUT_BIT (1 << 6)
#define MAX_ACK_WAIT 1.0
#define TESTER_ADDRESS 0x01
#define LED_CONTROL_REG 0x00
#define READ_REG_BASE 0x08
#define TEMP_SENSOR_ADDRESS 0x28
#define TEMP_DATA_MASK 0x3ff
#define TEMP_SIGN_THRESHOLD 512
#define TEMP_SCALE_FACTOR 4.0

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

void initI2c(void)
{
    portMask(0, PORT0_MASK);
    portMask(1, PORT1_MASK);
    portOut(0, 0x00);
    portOut(1, 0x00);
}

void writeI2c(int sda, int scl)
{
    unsigned char tempValue;
    tempValue = (sda << I2C1_SDA_OUT_SHIFT) | (scl << I2C1_SCL_OUT_SHIFT) | 0x07;
    tempValue |= I2C2_SDA_OUT_BIT | I2C2_SCL_OUT_BIT;
    portOut(0, tempValue);
}

void readI2c(int* sda, int* scl)
{
    unsigned char port0Value, port1Value;
    portIn(0, &port0Value);
    portIn(1, &port1Value);
    *sda = (port0Value >> I2C1_SDA_IN_SHIFT) & 1;
    *scl = port1Value & I2C1_SCL_IN_MASK;
}

void startI2c()
{
    writeI2c(1, 1);
    Delay(I2C_DELAY);
    writeI2c(0, 1);
    Delay(I2C_DELAY);
    writeI2c(0, 0);
    Delay(I2C_DELAY);
}

void stopI2c()
{
    writeI2c(0, 0);
    Delay(I2C_DELAY);
    writeI2c(0, 1);
    Delay(I2C_DELAY);
    writeI2c(1, 1);
    Delay(I2C_DELAY);
}

int getSda()
{
    unsigned char tempValue;
    portIn(0, &tempValue);
    return (tempValue >> I2C1_SDA_IN_SHIFT) & 0x01;
}

void sendBit(int bitValue)
{
    writeI2c(getSda(), 0);
    writeI2c(bitValue, 0);
    Delay(I2C_DELAY);
    writeI2c(bitValue, 1);
    Delay(I2C_DELAY);
    writeI2c(bitValue, 0);
    Delay(I2C_DELAY);
}

void sendByte(int byteValue)
{
    int i;
    for (i = 0; i < 8; i++)
        sendBit((byteValue >> (7 - i)) & 0x01);
}

int receiveAck()
{
    double elapsedTime = 0;
    writeI2c(getSda(), 0);
    Delay(I2C_DELAY);
    writeI2c(0, 0);
    Delay(I2C_DELAY);
    writeI2c(0, 1);
    while (getSda() == 1)
    {
        Delay(I2C_DELAY);
        elapsedTime += I2C_DELAY;
        if (elapsedTime > MAX_ACK_WAIT)
            return 1;
    }
    Delay(I2C_DELAY);
    writeI2c(0, 0);
    return 0;
}

void sendAck()
{
    sendBit(0);
}

void sendNack()
{
    sendBit(1);
}

int receiveBit()
{
    int bitValue;
    writeI2c(1, 0);
    Delay(I2C_DELAY);
    writeI2c(1, 1);
    Delay(I2C_DELAY);
    bitValue = getSda();
    Delay(I2C_DELAY);
    writeI2c(1, 0);
    return bitValue;
}

void receiveByte(int* byteValue)
{
    int i, tempValue = 0;
    for (i = 0; i < 8; i++)
    {
        tempValue |= (receiveBit() << (7 - i));
    }
    *byteValue = tempValue;
}

int writeWord(int address, int subAddress, int wordValue)
{
    restrictIntValue(&address, 0, 127);
    address <<= 1;
    restrictIntValue(&subAddress, 0, 255);
    restrictIntValue(&wordValue, 0, 255);
    startI2c();
    sendByte(address);
    if (receiveAck() != 0)
        return 1;
    sendByte(subAddress);
    if (receiveAck() != 0)
        return 1;
    sendByte(wordValue);
    if (receiveAck() != 0)
        return 1;
    stopI2c();
    return 0;
}

int readWord(int address, int subAddress, int* outputData, int byteCount)
{
    int i;
    restrictIntValue(&address, 0, 127);
    if (byteCount < 1)
        byteCount = 1;
    address <<= 1;
    restrictIntValue(&subAddress, 0, 255);
    startI2c();
    sendByte(address);
    if (receiveAck() != 0)
        return 1;
    sendByte(subAddress);
    if (receiveAck() != 0)
        return 1;
    stopI2c();
    startI2c();
    address |= 0x01;
    sendByte(address);
    if (receiveAck() != 0)
        return 1;
    for (i = 0; i < byteCount; i++)
    {
        receiveByte(outputData + i);
        if (i != (byteCount - 1))
            sendAck();
    }
    sendNack();
    stopI2c();
    return 0;
}

int readTemperature(double* temperatureValue)
{
    int tempValue, dataValue = 0;
    int address = TEMP_SENSOR_ADDRESS;
    address = (address << 1) | 0x01;
    startI2c();
    sendByte(address);
    if (receiveAck() != 0)
        return 1;
    receiveByte(&tempValue);
    sendAck();
    dataValue = tempValue << 2;
    receiveByte(&tempValue);
    sendNack();
    dataValue += (tempValue >> 6);
    stopI2c();
    dataValue = dataValue > TEMP_SIGN_THRESHOLD ? dataValue - 1024 : dataValue;
    *temperatureValue = (double)dataValue / TEMP_SCALE_FACTOR;
    return 0;
}