#include <SevSeg.h>

// объект для работы с дисплеем
SevSeg sevseg; 

// пин для подключения к батарейке
#define batteryPin A0 
// сигменты (аноды)
#define a 11
#define b 7
#define c 4
#define d 2
#define e 1
#define f 10
#define g 5
#define dp 3
// разрыды (катоды)
#define d1 12
#define d2 9
#define d3 8
#define d4 6

void setup() {
  // количество цифр на дисплее
  byte numDigits = 4;
  // пины для подключения к разрядам дисплея (D1, D2, D3, D4) 
  byte digitPins[] = {d1, d2, d3, d4};
  // пины для подключения к сегментам дисплея (a, b, c, d, e, f, g, dp)
  byte segmentPins[] = {a, b, c, d, e, f, g, dp};
  // инициализация библиотеки
  sevseg.begin(COMMON_CATHODE, numDigits, digitPins, segmentPins); 
  // установка яркости дисплея (0-100%)
  sevseg.setBrightness(90);
}

void loop() {
  // считываем значение напряжения с батарейки
  int batteryValue = analogRead(batteryPin); 
  // преобразуем значение в напряжение (5V - опорное напряжение; 10bit - дискретизация АЦП ардуино)
  float voltage = batteryValue * (5.0 / 1023.0); 
  // отображаем напряжение на дисплее
  displayVoltage(voltage); 
}

void displayVoltage(float voltage) {
  // получаем целую часть напряжения
  int volts = voltage;
  // получаем десятичную часть напряжения
  int decimals = (voltage - volts) * 100;
  // отображаем напряжение на дисплее
  sevseg.setNumber(volts * 100 + decimals, 2); 
  sevseg.refreshDisplay();
}