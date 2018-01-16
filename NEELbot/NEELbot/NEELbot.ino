#include <Herkulex.h>
#include <SoftwareSerial.h>

byte testMotor = 19;

void setup() {
  Serial.begin(57600);
  Herkulex.begin(57600,4,3);
  Herkulex.reboot(testMotor); 
  Herkulex.setLed(testMotor, 0x02);

  Herkulex.reboot(testMotor);


}

void loop() {

}




