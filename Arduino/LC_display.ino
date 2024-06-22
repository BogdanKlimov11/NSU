#include <LiquidCrystal.h>

// определяем пины для подключения дисплея
#define GND_PIN 22
#define VDD_PIN 24
#define VO_PIN 26
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

// инициализируем экземпляр LiquidCrystal
LiquidCrystal lcd(RS_PIN, EN_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);

void setup() {
  // инициализация пинов
  // устанавливаем питание как выход
  // устанавливаем землю как выход
  pinMode(VDD_PIN, OUTPUT);
  pinMode(GND_PIN, OUTPUT);
  pinMode(BLA_PIN, OUTPUT);
  pinMode(BLK_PIN, OUTPUT);
  pinMode(VO_PIN, OUTPUT);
  pinMode(RS_PIN, OUTPUT);
  pinMode(RW_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  // устанавливаем цифровой пин 
  pinMode(D0_PIN, INPUT);
  pinMode(D1_PIN, INPUT);
  pinMode(D2_PIN, INPUT);
  pinMode(D3_PIN, INPUT);
  pinMode(D4_PIN, INPUT);
  pinMode(D5_PIN, INPUT);
  pinMode(D6_PIN, INPUT);
  pinMode(D7_PIN, INPUT);

  // подаем напряжение на питание
  digitalWrite(VDD_PIN, HIGH);
  digitalWrite(BLA_PIN, HIGH);

  // соединяем с землей
  digitalWrite(BLK_PIN, LOW);
  digitalWrite(GND_PIN, LOW);
  
  // устанавливаем размер экрана (количество столбцов и строк)
  lcd.begin(16, 2);
}

void loop() {
  lcd.clear();
  digitalWrite(VO_PIN, LOW);

  // устанавливаем курсор на первую строку и нулевой символ
  lcd.setCursor(0, 0);

  // выводим текст на экран
  lcd.print("Hello, World!");
  
  delay(10);
  digitalWrite(VO_PIN, HIGH);
  lcd.clear();
}