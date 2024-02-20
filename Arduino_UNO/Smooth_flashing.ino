// пин для встроенного светодиода (13 на Arduino UNO)
int ledPin = 13;
// начальная яркость
int brightness = 0;
// величина изменения яркости
int fadeAmount = 1;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  megay();
}

void megay() {
  // включаем светодиод
  digitalWrite(ledPin, HIGH);
  // ждем в течение времени, равного текущей яркости
  delay(brightness);
  // выключаем светодиод
  digitalWrite(ledPin, LOW);
  // ждем оставшееся время
  delay(30 - brightness);
  // изменяем яркость
  brightness = brightness + fadeAmount;
  // если достигли минимальной или максимальной яркости, меняем направление изменения
  if (brightness <= 0 || brightness >= 30) {
    fadeAmount = -fadeAmount;
  }
}
