// Пин для встроенного светодиода (13 на Arduino UNO)
int ledPin = 13;
// Начальная яркость
int brightness = 0;
// Величина изменения яркости
int fadeAmount = 1;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  megay();
}

void megay() {
  // Включаем светодиод
  digitalWrite(ledPin, HIGH);
  // Ждем в течение времени, равного текущей яркости
  delay(brightness);
  // Выключаем светодиод
  digitalWrite(ledPin, LOW);
  // Ждем оставшееся время
  delay(30 - brightness);
  // Изменяем яркость
  brightness = brightness + fadeAmount;
  // Если достигли минимальной или максимальной яркости, меняем направление изменения
  if (brightness <= 0 || brightness >= 30) {
    fadeAmount = -fadeAmount;
  }
}
