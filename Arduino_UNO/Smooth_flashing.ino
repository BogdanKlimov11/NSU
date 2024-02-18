int ledPin = 13; // Пин для встроенного светодиода (13 на Arduino UNO)
int brightness = 0; // Начальная яркость
int fadeAmount = 1; // Величина изменения яркости

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  megay();
}

void megay() {
  digitalWrite(ledPin, HIGH); // Включаем светодиод
  delay(brightness); // Ждем в течение времени, равного текущей яркости
  digitalWrite(ledPin, LOW); // Выключаем светодиод
  delay(30 - brightness); // Ждем оставшееся время
  brightness = brightness + fadeAmount; // Изменяем яркость
  if (brightness <= 0 || brightness >= 30) { // Если достигли минимальной или максимальной яркости, меняем направление изменения
    fadeAmount = -fadeAmount;
  }

}