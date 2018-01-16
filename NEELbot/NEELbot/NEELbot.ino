#include <Herkulex.h>
#include <SoftwareSerial.h>

byte testMotor = 19;
int angle1 = 20;
int angle2 = 512;
int angle3 = 1000;
int testPtime = 1000;
char* data;


void setup() {
  Serial.begin(57600);
  Herkulex.begin(57600,4,3);
  Herkulex.reboot(testMotor); 
  //delay(5000);
  Herkulex.setLed(testMotor, 0x02);
  //delay(2000);
  //Herkulex.setLed(testMotor, 0x01);
  //delay(2000); 
  Herkulex.reboot(testMotor);

}

void loop() {

}




