void setup() {
  // инициализация последовательной связи
  Serial.begin(9600);
}

void loop() {
  // считываем значение с аналогового пина
  int sensorValue = analogRead(A0);
  // преобразуем значение в напряжение
  float voltage = sensorValue * (5.0 / 1023.0);
  // выводим значение в консоль
  Serial.print("Напряжение на батарейке: ");
  Serial.print(voltage);
  Serial.println(" Вольт");
  // задержка в 1 секунду
  delay(1000);
}