// пины строк
byte r[8] = {10, A3, 9, A1, 2, 8, 3, 6};
// пины столбцов
byte c[8] = {A2, 4, 5, 11, 7, A0, A4, A5};
// основание системы
int base = 12;

/* цифра 0 */
byte num0[8] = {
  0b00000000,
  0b00111100,
  0b01100110,
  0b01101110,
  0b01110110,
  0b01100110,
  0b00111100,
  0b00000000
};

/* цифра 1 */
byte num1[8] = {
  0b00000000,
  0b00011000,
  0b00111000,
  0b00011000,
  0b00011000,
  0b00011000,
  0b01111110,
  0b00000000
};

/* цифра 2 */
byte num2[8] = {
  0b00000000,
  0b00111100,
  0b01100110,
  0b00001100,
  0b00011000,
  0b00110000,
  0b01111110,
  0b00000000
};

/* цифра 3 */
byte num3[8] = {
  0b00000000,
  0b00111100,
  0b01100110,
  0b00001110,
  0b00000110,
  0b01100110,
  0b00111100,
  0b00000000
};

/* цифра 4 */
byte num4[8] = {
  0b00000000,
  0b01100110,
  0b01100110,
  0b01111110,
  0b01111110,
  0b00000110,
  0b00000110,
  0b00000000
};

/* цифра 5 */
byte num5[8] = {
  0b00000000,
  0b01111110,
  0b01100000,
  0b01111100,
  0b00000110,
  0b00000110,
  0b01111100,
  0b00000000
};

/* цифра 6 */
byte num6[8] = {
  0b00000000,
  0b00111100,
  0b01100110,
  0b01100000,
  0b01111100,
  0b01100110,
  0b00111100,
  0b00000000
};

/* цифра 7 */
byte num7[8] = {
  0b00000000,
  0b01111110,
  0b00000110,
  0b00001100,
  0b00011000,
  0b00110000,
  0b01100000,
  0b00000000
};

/* цифра 8 */
byte num8[8] = {
  0b00000000,
  0b00111100,
  0b01100110,
  0b00111100,
  0b01100110,
  0b01100110,
  0b00111100,
  0b00000000
};

/* цифра 9 */
byte num9[8] = {
  0b00000000,
  0b00111100,
  0b01100110,
  0b00111110,
  0b00000110,
  0b01100110,
  0b00111100,
  0b00000000
};

/* цифра A */
byte numA[8] = {
  0b00000000,
  0b00111100,
  0b01100110,
  0b01100110,
  0b01111110,
  0b01100110,
  0b01000010,
  0b00000000
};

/* цифра B */
byte numB[8] = {
  0b00000000,
  0b01111100,
  0b01100110,
  0b01111100,
  0b01100110,
  0b01100110,
  0b01111100,
  0b00000000
};

/* цифра C */
byte numC[8] = {
  0b00000000,
  0b00111110,
  0b01100000,
  0b01100000,
  0b01100000,
  0b01100000,
  0b00111110,
  0b00000000
};

/* цифра D */
byte numD[8] = {
  0b00000000,
  0b01111100,
  0b01100110,
  0b01100110,
  0b01100110,
  0b01100110,
  0b01111100,
  0b00000000
};

/* цифра E */
byte numE[8] = {
  0b00000000,
  0b01111110,
  0b01100000,
  0b01111110,
  0b01100000,
  0b01100000,
  0b01111110,
  0b00000000
};

/* цифра F */
byte numF[8] = {
  0b00000000,
  0b01111110,
  0b01100000,
  0b01111110,
  0b01100000,
  0b01100000,
  0b01100000,
  0b00000000
};

byte* bin; // указатель на массив с кадром

void setup() {
  for (byte i = 0; i < 8; i++) { // инициализация пинов
    pinMode(r[i], OUTPUT);
    digitalWrite(r[i], LOW);
    pinMode(c[i], OUTPUT);
    digitalWrite(c[i], HIGH);
  }
}

void loop() {
  // вывод активного кадра
  view(bin);
  // смена активного кадра каждую секунду
  change(base);
}

void view(byte* n) {
  static unsigned long timer;
  static byte ii = 0;
  static byte ii_old = 0;
  if (timer > millis()) return;
  digitalWrite(c[ii_old], HIGH);
  byte bb = 0b00000001;
  // перебираем биты, выводим в пины
  for (byte i = 0; i < 8; i++) {
    digitalWrite(r[i], (bb & *(n + ii)));
    bb = bb << 1;
  }
  digitalWrite(c[ii], LOW);
  ii_old = ii;
  ii = ii == 7 ? 0 : ii + 1;
  timer = millis() + 1;
}

void change(int base_digit) {
  static byte nom = 0;
  static unsigned long timer;
  if (timer > millis()) {
    return;
  }
  // циклически меняем активный кадр каждую секунду
  switch (nom) {
    case 0:
      bin = &num0[0];
      break;
    case 1:
      bin = &num1[0];
      break;
    case 2:
      bin = &num2[0];
      break;
    case 3:
      bin = &num3[0];
      break;
    case 4:
      bin = &num4[0];
      break;
    case 5:
      bin = &num5[0];
      break;
    case 6:
      bin = &num6[0];
      break;
    case 7:
      bin = &num7[0];
      break;
    case 8:
      bin = &num8[0];
      break;
    case 9:
      bin = &num9[0];
      break;
    case 10:
      bin = &numA[0];
      break;
    case 11:
      bin = &numB[0];
      break;
    case 12:
      bin = &numC[0];
      break;
    case 13:
      bin = &numD[0];
      break;
    case 14:
      bin = &numE[0];
      break;
    case 15:
      bin = &numF[0];
      break;
  }
  nom++;
  nom = nom % base_digit;
  timer = millis() + 1000;
}