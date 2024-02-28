// смещениен нуля
float k1 = 0.02;
// смещение опорного нуля
float k2 = 5.15;
// диапозон работы Arduino UNO
float k = k2 - k1;

void setup() {
  // инициализация последовательной связи
  Serial.begin(9600);
}

void loop() {
  // считываем значение с аналогового пина
  int sensorValue = analogRead(A0);
  // преобразуем значение в напряжение
  float voltage = sensorValue * (k / 1023.0);
  // выводим значение в консоль
  Serial.print("Напряжение на батарейке: ");
  Serial.print(voltage);
  Serial.println(" Вольт");
  // задержка в 1 секунду
  delay(1000);
}
