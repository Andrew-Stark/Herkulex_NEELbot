#include <Herkulex.h>
#include <SoftwareSerial.h>

byte testMotor = 19;
int angle1 = 20;
int angle2 = 512;
int angle3 = 1000;
int testPtime = 1000;
char* data;


void setup() {

  Herkulex.begin(115200,4,3);
  Herkulex.reboot(testMotor);
  delay(2000);
  Herkulex.setLed(testMotor, LED_GREEN);

  //delay(2000);

  

  //delay(2000);

  Herkulex.setLed(testMotor, LED_BLUE);


  Herkulex.reboot(testMotor);
  
  
}

void loop() {

}




