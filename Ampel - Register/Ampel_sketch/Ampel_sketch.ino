#define PINLEDGREEN 2
#define PINLEDYELLOW 3
#define PINLEDRED 4

void setup() {
  // Pins 2, 3 und 4 als Outputs
  for (int i = 2; i < 5; i++) {
    pinMode(i, OUTPUT);
  }
}

void green(void);
void red(void);
void yellow(void);
void clear(void);

void loop() {
  clear();      // Alle Pins auf Low
  green();      // Grün an
  delay(5000);  // Warten

  clear();
  yellow();
  delay(2000);

  clear();
  red();
  delay(5000);

  clear();
  yellow();
  red();
  delay(1000);
}

// Funktionen der Ampelphasen
void green(void) {
  digitalWrite(PINLEDGREEN, HIGH);
}
void yellow(void) {
  digitalWrite(PINLEDYELLOW, HIGH);
}
void red(void) {
  digitalWrite(PINLEDRED, HIGH);
}

// Fkt für alle Pins auf Low
void clear(void) {
  for (int i = 2; i < 5; i++) {
    digitalWrite(i, LOW);
  }
}
