#include <SoftwareSerial.h>

SoftwareSerial SSerial(4,3);
void setup(){
 
  // put your setup code here, to run once:
  SSerial.begin(115200);
  herkReboot(19);
  delay(1000);
  herkReboot(19);
}

void loop() {
  // put your main code here, to run repeatedly:

}
