#include <SoftwareSerial.h>

SoftwareSerial SSerial(4,3);
void setup(){
 
  pinMode(5, OUTPUT);
  SSerial.begin(57600);
  Serial.begin(57600);
  set_led(19, 1);
  
  
}

void loop() {
  digitalWrite(5, HIGH);
  delay(1000);
  digitalWrite(5, LOW);
  delay(1000);

}


