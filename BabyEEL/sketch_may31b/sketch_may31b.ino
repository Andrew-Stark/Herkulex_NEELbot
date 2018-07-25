byte servo0 = B00000100;  //PORT D bit 2
byte servo1 = B00001000;  //PORT D bit 3
byte servo2 = B00010000;  //PORT D bit 4
byte servo3 = B00100000;  //PORT D bit 5
byte servo4 = B01000000;  //PORT D bit 6
byte servo5 = B10000000;  //PORT D bit 7

byte servoAll = B11111100; //all six port D

int high = 2000;
int low  = 600;
int period = 40;

void setup() {
  DDRD |= B11111100;
  
  PORTD &= 0;
}

void loop() {
  for(int i = low; i <= high; i+=period){
    PORTD |= servoAll;
    delayMicroseconds(i);
    PORTD &= ~servoAll;
    delay(10);
  }
  for(int i = high; i >= low; i-=period){
    PORTD |= servoAll;
    delayMicroseconds(i);
    PORTD &= ~servoAll;
    delay(10);
  }  

}



