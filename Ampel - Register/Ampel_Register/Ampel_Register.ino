// Taktfrequenz einstellen
#define F_CPU 16000000UL
// Bibliotheken
#include <avr/io.h>
#include <util/delay.h>


void myInit(void);
void myLoop(void);
void green(void);
void red(void);
void yellow(void);
void clear(void);

int main(void){
  mySetup();
  while (1) {
    myLoop();
  }
  return 0;
}

void mySetup() {
// Pins 2, 3 und 4 als Outputs
  DDRD |= (1 << DDB2);
  DDRD |= (1 << DDB3);
  DDRD |= (1 << DDB4);
}

void myLoop() {
  clear(); // Alle Pins auf Low
  green(); // Grün an
  _delay_ms(5000); // Warten

  clear();
  yellow();
  _delay_ms(2000);

  clear();
  red();
  _delay_ms(5000);

  clear();
  yellow();
  red();
  _delay_ms(1000);
}

// Funktionen der Ampelphasen
void green(void){
  PORTD |= (1 << PORTD2);
}
void yellow(void){
  PORTD |= (1 << PORTD3);
}
void red(void){
  PORTD |= (1 << PORTD4);
}

// Fkt für alle Pins auf Low
void clear(void){
  PORTD &= ~(1 << PORTD2);
  PORTD &= ~(1 << PORTD3);
  PORTD &= ~(1 << PORTD4);
}
