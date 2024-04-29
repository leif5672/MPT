//Comunication between ATMEGA823P & STM32 
#include<stdint.h>


//Pin numbers
  //OUTPUTS
    uint8_t LED1 = 7;
    uint8_t LED2 = 8; 
    uint8_t LED3 = 9;
    uint8_t LED4 = 10;
  
  
  //INPUTS
    uint8_t CLK_Signal = 2;
    uint8_t Bit1_Pin = 12;
  

   uint8_t counter = 0;
   uint8_t value = 0;
  
void setup() {
  
  
   //CLK Signal
  pinMode(CLK_Signal, INPUT);
   // Bit 1
  pinMode(Bit1_Pin, INPUT);

  
   //LED 1
  pinMode(LED1, OUTPUT);
   //LED 2
  pinMode(LED2, OUTPUT);
  //LED 3
  pinMode(LED3, OUTPUT);
  //LED 4
  pinMode(LED4, OUTPUT);

  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(CLK_Signal),readinput,FALLING);
  
}

void loop() {
  // put your main code here, to run repeatedly:
 //digitalWrite(LED1, HIGH);
 //delay(1000);
 //digitalWrite(LED2, HIGH);
 //delay(1000);
 //digitalWrite(LED3, HIGH);
 //delay(1000);
 //digitalWrite(LED4, HIGH);

//  Serial.println(value);
//delay(1000); 



}

void readinput () {
  
  counter ++;

  value = value + !digitalRead(Bit1_Pin);
  if(counter < 4){
  value = value << 1;

  value = value & 0x0F;
  }
Serial.println(counter);
  Serial.println(value);
  


  if (counter == 4) {
    counter = 0;

    digitalWrite(LED1, bitRead(value,0));
    digitalWrite(LED2, bitRead(value,1));
    digitalWrite(LED3, bitRead(value,2));     
    digitalWrite(LED4, bitRead(value,3));
    value = 0;
    Serial.println("Hello world!1");
  }
}
