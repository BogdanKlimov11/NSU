void setup() {               
  pinMode(13, OUTPUT);   
}
 
void loop() {
  // зажигаем светодиод
  digitalWrite(13, HIGH);
  // ждем секунду
  delay(1000);
  // выключаем светодиод
  digitalWrite(13, LOW);
  // ждем секунду
  delay(1000);
}
