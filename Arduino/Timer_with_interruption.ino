#include <LiquidCrystal.h>

int time = 0, hours, seconds, minutes;
bool led_state = false;

#define GND_PIN 22
#define VDD_PIN 24
#define V0_PIN 26
#define RS_PIN 28
#define RW_PIN 30
#define EN_PIN 32
#define D0_PIN 34
#define D1_PIN 36
#define D2_PIN 38
#define D3_PIN 40
#define D4_PIN 42
#define D5_PIN 44
#define D6_PIN 46
#define D7_PIN 48
#define BLA_PIN 50
#define BLK_PIN 52

LiquidCrystal lcd(RS_PIN, EN_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);

void setup() {
  Serial.begin(9600);

  pinMode(VDD_PIN, OUTPUT);
  pinMode(GND_PIN, OUTPUT);
  pinMode(BLA_PIN, OUTPUT);
  pinMode(BLK_PIN, OUTPUT);
  pinMode(V0_PIN, OUTPUT);
  pinMode(RS_PIN, OUTPUT);
  pinMode(RW_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(D0_PIN, INPUT);
  pinMode(D1_PIN, INPUT);
  pinMode(D2_PIN, INPUT);
  pinMode(D3_PIN, INPUT);
  pinMode(D4_PIN, INPUT);
  pinMode(D5_PIN, INPUT);
  pinMode(D6_PIN, INPUT);
  pinMode(D7_PIN, INPUT);

  digitalWrite(BLA_PIN, HIGH);
  digitalWrite(BLK_PIN, LOW);
  digitalWrite(GND_PIN, LOW);
  digitalWrite(VDD_PIN, HIGH);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);

  // инициализация Timer1
  // отключить глобальные прерывания
  cli();
  // установить регистры в 0
  TCCR1A = 0;
  TCCR1B = 0;

  // установка регистра совпадения
  OCR1A = 15624;

  // включить CTC режим 
  TCCR1B |= (1 << WGM12);
  // установить биты на коэффициент деления 1
  TCCR1B |= (1 << CS10) | (1 << CS12);

  // включить прерывание по совпадению таймера
  TIMSK1 |= (1 << OCIE1A);

  // установить регистры в 0
  TCCR2A = 0;
  TCCR2B = 0;

  // установка регистра совпадения
  OCR2A = 15624;

  // включить CTC режим 
  TCCR2B |= (1 << WGM12);
  // установить биты на коэффициент деления 1
  TCCR2B |= (1 << CS10);

  // включить прерывание по совпадению таймера 
  TIMSK2 |= (1 << OCIE1A);

  // включить глобальные прерывания
  sei();
}

void loop() {

}

ISR(TIMER2_COMPA_vect) {
  if(led_state)
  digitalWrite(V0_PIN, HIGH);
  else
  digitalWrite(V0_PIN, LOW);

  led_state = !led_state;
}

ISR(TIMER1_COMPA_vect) {
  lcd.clear();

  time++;
  hours = time/3600;
  minutes = time/60;
  seconds = time;

  if(hours>12) {
  hours = hours % 60;
  }

  if(minutes >= 60) {
  minutes = minutes % 60;
  }

  if(seconds >= 60) {
  seconds = seconds % 60;
  }

  if(hours >= 10) {
  lcd.setCursor(0, 0);
  lcd.print(hours);
  }
  else {
  lcd.setCursor(0, 0);
  lcd.print("0");
  lcd.setCursor(1, 0);
  lcd.print(hours);
  }

  lcd.setCursor(2, 0);
  lcd.print(":");

  if(minutes >= 10) {
  lcd.setCursor(3, 0);
  lcd.print(minutes);
  }
  else {
  lcd.setCursor(3, 0);
  lcd.print("0");
  lcd.setCursor(4, 0);
  lcd.print(minutes);
  }

  lcd.setCursor(5, 0);
  lcd.print(":");

  if(seconds >= 10) {
  lcd.setCursor(6, 0);
  lcd.print(seconds);
  }
  else {
  lcd.setCursor(6, 0);
  lcd.print("0");
  lcd.setCursor(7, 0);
  lcd.print(seconds%10);
  }
}
