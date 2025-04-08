#include <SevSeg.h>

SevSeg sevseg;

// аналоговый пин для измерения напряжения
#define batteryPin A0

// пин для разрядки (через резистор/транзистор)
#define dischargePin 13

// сегменты (аноды)
#define a 11
#define b 7
#define c 4
#define d 2
#define e 1
#define f 10
#define g 5
#define dp 3
// разряды (катоды)
#define d1 12
#define d2 9
#define d3 8
#define d4 6

// для отслеживания времени
unsigned long startTime;

void setup() {
  Serial.begin(9600);
  pinMode(dischargePin, OUTPUT);
  digitalWrite(dischargePin, LOW);
  
  byte numDigits = 4;
  byte digitPins[] = {d1, d2, d3, d4};
  byte segmentPins[] = {a, b, c, d, e, f, g, dp};
  sevseg.begin(COMMON_CATHODE, numDigits, digitPins, segmentPins);
  sevseg.setBrightness(90);
  
  startTime = millis();
}

void loop() {
  int batteryValue = analogRead(batteryPin);
  float voltage = batteryValue * (5.0 / 1023.0);
  
  // отправляем данные в Serial (время в секундах, напряжение)
  unsigned long currentTime = millis() - startTime;
  float timeSeconds = currentTime / 1000.0;
  
  Serial.print(timeSeconds, 3);
  Serial.print(",");
  Serial.println(voltage, 3);
  
  // выводим напряжение на дисплей
  displayVoltage(voltage);

  // задержка для плавного графика
  delay(100);
}

void displayVoltage(float voltage) {
  int volts = voltage;
  int decimals = (voltage - volts) * 100;
  sevseg.setNumber(volts * 100 + decimals, 2);
  sevseg.refreshDisplay();
}
