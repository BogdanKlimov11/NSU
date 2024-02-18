void setup() {               
  pinMode(13, OUTPUT);   
}
 
void loop() {
  digitalWrite(13, HIGH);   // зажигаем светодиод
  delay(1000);              // ждем секунду
  digitalWrite(13, LOW);    // выключаем светодиод
  delay(1000);              // ждем секунду
}
