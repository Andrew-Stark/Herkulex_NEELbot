#include <Herkulex.h>
//#include <SoftwareSerial.h>

int testMotor = 19;
int angle1 = 20;
int angle2 = 512;
int angle3 = 1000;
int testPtime = 1000;
char* data;


void setup() {
  Serial.begin(115200);
  Herkulex.reboot(testMotor);
  //delay(250);
  Herkulex.setLed(testMotor, LED_GREEN);

  //delay(2000);

  

  delay(2000);

  Herkulex.setLed(testMotor, LED_BLUE);

  delay(2000);
  Herkulex.reboot(testMotor);
  
  
}

void loop() {

}




