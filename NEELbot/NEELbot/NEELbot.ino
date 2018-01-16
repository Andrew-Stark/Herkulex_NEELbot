#include <Herkulex.h>
#include <SoftwareSerial.h>

byte testMotor = 19;

void setup() {
  pinMode(5,OUTPUT);
  delay(2000);
  digitalWrite(5,HIGH);
  delay(2000);
  Serial.begin(57600);
  Herkulex.begin(57600,4,3);
  Herkulex.reboot(testMotor); 
//  for(int i=0;i<10000;i++){
//    for(byte i=1;i<10;i++){
//      Herkulex.setLed(testMotor, i);
//    }
//  }
  Herkulex.setLed(testMotor,1);   
  delay(10000);
  

  Herkulex.reboot(testMotor);

  digitalWrite(5,LOW);


}

void loop() {

}




