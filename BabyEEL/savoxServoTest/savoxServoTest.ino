#include <Servo.h>

Servo servo_0;
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;

int pos = 0;

void setup() {
  servo_0.attach(3);
  servo_1.attach(5);
  servo_2.attach(6);
  servo_3.attach(9);
  servo_4.attach(10);
  servo_5.attach(11);
}

void loop() {
//000000000000000000000000000000000000000000000000
  for(pos=0;pos<=180;pos+=1){
    servo_0.write(pos);
    delay(15);
  }
  for(pos=180;pos>=90;pos-=1){
    servo_0.write(pos);
    delay(15);
  }
//1111111111111111111111111111111111111111111111111
  for(pos=0;pos<=180;pos+=1){
    servo_1.write(pos);
    delay(15);
  }
  for(pos=180;pos>=90;pos-=1){
    servo_1.write(pos);
    delay(15);
  }
//22222222222222222222222222222222222222222222222222
  for(pos=0;pos<=180;pos+=1){
    servo_2.write(pos);
    delay(15);
  }
  for(pos=180;pos>=90;pos-=1){
    servo_2.write(pos);
    delay(15);
  }
//333333333333333333333333333333333333333333333333333
  for(pos=0;pos<=180;pos+=1){
    servo_3.write(pos);
    delay(15);
  }
  for(pos=180;pos>=90;pos-=1){
    servo_3.write(pos);
    delay(15);
  }
//444444444444444444444444444444444444444444444444444
  for(pos=0;pos<=180;pos+=1){
    servo_4.write(pos);
    delay(15);
  }
  for(pos=180;pos>=90;pos-=1){
    servo_4.write(pos);
    delay(15);
  }
//5555555555555555555555555555555555555555555555555555
  for(pos=0;pos<=180;pos+=1){
    servo_5.write(pos);
    delay(15);
  }
  for(pos=180;pos>=90;pos-=1){
    servo_5.write(pos);
    delay(15);
  }
  
}
