#include <Herkulex.h>
#include <SoftwareSerial.h>

int testMotor = 19;
int angle1 = 20;
int angle2 = 512;
int angle3 = 1000;
int testPtime = 1000;
char* data;


void setup() {
  delay(1000);
  Serial.begin(115200);
  //Herkulex.begin(115200,0,1); 
  delay(250);
  Herkulex.reboot(testMotor);

  //Herkulex.initialize();
  Herkulex.torqueON(testMotor);

  Herkulex.moveOne(testMotor, angle3, testPtime, LED_GREEN);

  //int pos = Herkulex.getPosition(testMotor);
  //Serial.println(pos);

  Herkulex.setLed(19, LED_GREEN);

  delay(2000);
  Herkulex.reboot(testMotor);
  
  
}

void loop() {

}




