/*
 Hekulex.cpp - Library for Dongbu Herkulex DRS-0101/DRS-0201 
 Copyright (c) 2012 - http://robottini.altervista.org
 Created by Alessandro on 09/12/2012.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,  
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 *****************************************************************************
    PLEASE START READING: Herkulex Servo Manual (http://www.hovis.co.kr/guide/herkulexeng.pdf)
 *****************************************************************************
 
 IMPORTANT:

  The library works on Arduino UNO/2009 - Arduino Mega.
  Please with Arduino UNO/2009 works with SoftwareSerial library modified with baud rate 57.600.
  Use this begin type:
		begin(57600, int rx, int tx);
 
  For Arduino Mega, please use baud rate 115.200

 *****************************************************************************
 Contact: alegiaco@gmail.com
 Web:     http://robottini.altervista.org
 Autor:   Alessandro Giacomel
 *****************************************************************************  
 
	01/01/2018	- Simplifying library for use with NEELbot.
			- Intended changes:
				write "send package" function to replace specialized internals of Herkulex.MEMBER functions, improve readability and reliability
				add member function to handle writing different motions to all motors, simultaneous movement.
				include capability to read 40 x 19 motion array
				will perhaps need to write program (python, excel, matlab) to convert ASCII text files (in degrees) to binary files for creating arrays.

*/
#include "Herkulex.h"
#include <SoftwareSerial.h>
#include <stdio.h>


// Macro for the Serial port selection
#define HSerial1     1 		// Write in Serial 1 port Arduino NANO - Pin 19(rx) - 18 (tx) 
#define HSerial2     2   	// Write in Serial 2 port Arduino Mega - Pin 17(rx) - 16 (tx) 
#define HSerial3     3   	// Write in Serial 3 port Arduino Mega - Pin 15(rx) - 14 (tx)
#define SSerial      4   	// Write in SoftSerial Arduino with 328p or Mega
 
extern SoftwareSerial SwSerial(4,3);

// Herkulex begin with Arduino Uno
void HerkulexClass::begin(long baud, int rx, int tx)
{
	SwSerial.setRX(rx);
	SwSerial.setTX(tx);
	SwSerial.begin(baud);
	port = SSerial;
}

#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
// Herkulex begin with Arduino Mega - Serial 1
void HerkulexClass::beginSerial1(long baud)
{
	Serial1.begin(baud);
	port = HSerial1;
}

// Herkulex begin with Arduino Mega - Serial 2
void HerkulexClass::beginSerial2(long baud)
{
	Serial2.begin(baud);
	port=HSerial2;
}

// Herkulex begin with Arduino Mega - Serial 3
void HerkulexClass::beginSerial3(long baud)
{
	Serial3.begin(baud);
	port = HSerial3;
}
#endif

// Herkulex end
void HerkulexClass::end()
{
	switch (port)
	{
	case SSerial:
		SwSerial.end();
		break;
// add #if defined (__AVR_ATmega328__) if possible for Arduino nano 328, need to find serial port data for this chip, 
    #if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__) || defined (__AVR_ATmega328__)
	case HSerial1:
		Serial1.end();
		break;
	case HSerial2:
		Serial2.end();
		break;
	case HSerial3:
		Serial3.end();
		break;
	#endif
	}
}

// initialize servos
void HerkulexClass::initialize()
{
        conta=0;
		lengthString=0;
        delay(100);       
        clearError(BROADCAST_ID);	// clear error for all servos
        delay(10);
        ACK(1);						// set ACK
        delay(10);
        torqueON(BROADCAST_ID);		// torqueON for all servos
        delay(10);
		
}

// stat
byte HerkulexClass::stat(int servoID)
{
	{
	pSize    = 0x07;			//3.Packet size
	pID      = servoID;			//4.Servo ID - 0XFE=All servos
	cmd      = HSTAT;			//5.CMD
	
	ck1=(pSize^pID^cmd)&0xFE;
        ck2=(~(pSize^pID^cmd))&0xFE ; 
  
	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	     
	sendData(dataEx, pSize);
	delay(2);
	readData(9); 				// read 9 bytes from serial

	
	pSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];
    data[1]=dataEx[8];
    lengthString=2;

	
    ck1 = (dataEx[2]^dataEx[3]^dataEx[4]^dataEx[7]^dataEx[8]) & 0xFE;
	ck2=checksum2(ck1);			
	
	if (ck1 != dataEx[5]) return -1; //checksum verify
	if (ck2 != dataEx[6]) return -2;

	return dataEx[7];			// return status
}
}

// torque on - 
void HerkulexClass::torqueON(int servoID)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = servoID;            // 4. Servo ID
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=0x60;               // 10. 0x60=Torque ON
	lengthString=3;             // lengthData
  	
	ck1=checksum1(data,lengthString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Torque ON

	sendData(dataEx, pSize);
}

// torque off - the torque is FREE, not Break
void HerkulexClass::torqueOFF(int servoID)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = servoID;            // 4. Servo ID
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=0x00;               // 10. 0x00=Torque Free
	lengthString=3;             // lengthData
  	
	ck1=checksum1(data,lengthString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Torque Free

    sendData(dataEx, pSize);

}

// ACK  - 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
void HerkulexClass::ACK(int valueACK)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = 0xFE;	            // 4. Servo ID
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=valueACK;           // 10.Value. 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
	lengthString=3;             // lengthData
  	
	ck1=checksum1(data,lengthString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value

 	sendData(dataEx, pSize);
}

// model - 1=0101 - 2=0201
byte HerkulexClass::model()
{
	pSize = 0x09;               // 3.Packet size 7-58
	pID   = 0xFE;	            // 4. Servo ID
	cmd   = HEEPREAD;           // 5. CMD
	data[0]=0x00;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	lengthString=2;             // lengthData
  	
	ck1=checksum1(data,lengthString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address
	dataEx[8] = data[1]; 		// Length

    sendData(dataEx, pSize);

	delay(1);
	readData(9);
	
	pSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];           // 8. 1st byte
	lengthString=1;              // lengthData
  	
	ck1=checksum1(data,lengthString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	if (ck1 != dataEx[5]) return -1; //checksum verify
	if (ck2 != dataEx[6]) return -2;
		
	return dataEx[7];			// return status

}

// setID - Need to restart the servo
void HerkulexClass::set_ID(int ID_Old, int ID_New)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = ID_Old;		        // 4. Servo ID OLD - original servo ID
	cmd   = HEEPWRITE;          // 5. CMD
	data[0]=0x06;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=ID_New;             // 10. ServoID NEW
	lengthString=3;             // lengthData
  	
	ck1=checksum1(data,lengthString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value

	sendData(dataEx, pSize);

}

// clearError
void HerkulexClass::clearError(int servoID)
{
	pSize = 0x0B;               // 3.Packet size 7-58
	pID   = servoID;     		// 4. Servo ID - 253=all servos
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x30;               // 8. Address
	data[1]=0x02;               // 9. Lenght
	data[2]=0x00;               // 10. Write error=0
	data[3]=0x00;               // 10. Write detail error=0
	
	lengthString=4;             // lengthData
  	
	ck1=checksum1(data,lengthString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value1
	dataEx[10]= data[3]; 		// Value2

	sendData(dataEx, pSize);
}

// move all servo at the same time to a position: servo list building
void HerkulexClass::moveAll(int servoID, int Goal, int iLed)
{
	  if (Goal > 1023 || Goal < 0)
		return;						 //0 <--> 1023 range
	  
	  int iMode=0;                   //mode=position
	  int iStop=0;                   //stop=0
	  

	  // Position definition
	  int posLSB=Goal & 0X00FF;					// MSB Pos
	  int posMSB=(Goal & 0XFF00) >> 8;			// LSB Pos

	  //led 
	  int iBlue=0;
	  int iGreen=0;
	  int iRed=0;
	  switch (iLed) {
	  case 1:
		iGreen=1;
		break;
	  case 2:
		iBlue=1;
		break;
	  case 3:
		iRed=1;
		break;
	  }
	  
	  int SetValue=iStop+iMode*2+iGreen*4+iBlue*8+iRed*16;	//assign led value

	  addData(posLSB, posMSB, SetValue, servoID);	//add servo data to list, pos mode
}

// move all servo at the same time to a position: servo list building
void HerkulexClass::moveAllAngle(int servoID, float angle, int iLed)
{
		if (angle > 160.0|| angle < -160.0) return; // out of the range	
		int position = (int)(angle/0.325) + 512;
		moveAll(servoID, position, iLed);
}

// move all servo at the same time with different speeds: servo list building
void HerkulexClass::moveSpeedAll(int servoID, int Goal, int iLed)
{
	  if (Goal > 1023 || Goal < -1023)
		return;								 //-1023 <--> 1023 range

	  int iMode=1;                  		// mode=continous rotation
	  int iStop=0;                  		// Stop=0

	  // Speed definition
	  int GoalSpeedSign;
	  if (Goal < 0) {
		GoalSpeedSign = (-1)* Goal ;
		GoalSpeedSign |= 0x4000;  //bit n°14 
	  } 
	  else {
		GoalSpeedSign = Goal;
	  }

	  int speedGoalLSB=GoalSpeedSign & 0X00FF; 	      		 // MSB speedGoal 
	  int speedGoalMSB=(GoalSpeedSign & 0xFF00) >> 8;        // LSB speedGoal 

	  //led 
	  int iBlue=0;
	  int iGreen=0;
	  int iRed=0;
	  switch (iLed) {
	  case 1:
		iGreen=1;
		break;
	  case 2:
		iBlue=1;
		break;
	  case 3:
		iRed=1;
		break;
	  }

	  int SetValue=iStop+iMode*2+iGreen*4+iBlue*8+iRed*16;	//assign led value

	  addData(speedGoalLSB, speedGoalMSB, SetValue, servoID);		//add servo data to list, speed mode
}

// move all servo with the same execution time
void HerkulexClass::actionAll(int pTime)
{
	if ((pTime <0) || (pTime > 2856)) return;

    pSize = 0x08 + conta;     	    // 3.Packet size 7-58
	cmd   = HSJOG;		 			// 5. CMD SJOG Write n servo with same execution time
	playTime=int((float)pTime/11.2);// 8. Execution time
 
    pID=0xFE^playTime;
    ck1=checksum1(moveData,conta);	//6. Checksum1
	ck2=checksum2(ck1);				//7. Checksum2

    pID=0xFE;
	dataEx[0] = 0xFF;				// Packet Header
	dataEx[1] = 0xFF;				// Packet Header	
	dataEx[2] = pSize;	 			// Packet Size
	dataEx[3] = pID;				// Servo ID
	dataEx[4] = cmd;				// Command Ram Write
	dataEx[5] = ck1;				// Checksum 1
	dataEx[6] = ck2;				// Checksum 2
	dataEx[7] = playTime;			// Execution time	
	
	for (int i=0; i < conta; i++)
		dataEx[i+8]=moveData[i];	// Variable servo data

	sendData(dataEx, pSize);

	conta=0; 						//reset counter   

}

// get Position
 int HerkulexClass::getPosition(int servoID) {
	int Position  = 0;

    pSize = 0x09;               // 3.Packet size 7-58
	pID   = servoID;     	    // 4. Servo ID - 253=all servos
	cmd   = HRAMREAD;           // 5. CMD
	data[0]=0x3A;               // 8. Address
	data[1]=0x02;               // 9. Lenght
	
	lengthString=2;             // lengthData
  	
	ck1=checksum1(data,lengthString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0];      	// Address  
	dataEx[8] = data[1]; 		// Length
	
	sendData(dataEx, pSize);

    delay(1);
	readData(13);

        	
	pSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];
    data[1]=dataEx[8];
    data[2]=dataEx[9];
    data[3]=dataEx[10];
    data[4]=dataEx[11];
    data[5]=dataEx[12];
    lengthString=6;

    ck1=checksum1(data,lengthString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

    if (ck1 != dataEx[5]) return -1;
	if (ck2 != dataEx[6]) return -1;

	Position = ((dataEx[10]&0x03)<<8) | dataEx[9];
        return Position;
	
}

float HerkulexClass::getAngle(int servoID) {
	int pos = (int)getPosition(servoID);
	return (pos-512) * 0.325;
}

// reboot single servo - pay attention 253 - all servos doesn't work!
// STARK - change should allow all servos, have not tested (1/1/18)
void HerkulexClass::reboot(byte servoID) {
        
	if((servoID < 0)||(servoID > 254))return;
	// data: NULL 
	byte dummyData[1] = {'\0'};
	createPacket(servoID, HREBOOT, dummyData);
	clearBuffer();	
	delay(200);	
}
//
// createPacket( MOTOR ID, Command, data)
// LED  - see table of colors 
void HerkulexClass::setLed(byte servoID, byte valueLed)
{
	if((valueLed < 1)||(valueLed > 9))return;
	if((servoID < 0)||(servoID > 254))return;
	// data: Address, Length, Color, NULL
	byte realData[4] = {0x35,0x01, valueLed, '\0'};
	createPacket(servoID, 0x03, realData); 
	clearBuffer();
}

// get the speed for one servo - values betweeb -1023 <--> 1023
int HerkulexClass::getSpeed(int servoID) {
  int speedy  = 0;

  pSize = 0x09;               // 3.Packet size 7-58
  pID   = servoID;     	   	  // 4. Servo ID 
  cmd   = HRAMREAD;           // 5. CMD
  data[0]=0x40;               // 8. Address
  data[1]=0x02;               // 9. Lenght

  lengthString=2;             // lengthData

  ck1=checksum1(data,lengthString);		//6. Checksum1
  ck2=checksum2(ck1);					//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header	
  dataEx[2] = pSize;		// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 	    // Address  
  dataEx[8] = data[1]; 		// Length

  sendData(dataEx, pSize);

  delay(1);
  readData(13);


  pSize = dataEx[2];           // 3.Packet size 7-58
  pID   = dataEx[3];           // 4. Servo ID
  cmd   = dataEx[4];           // 5. CMD
  data[0]=dataEx[7];
  data[1]=dataEx[8];
  data[2]=dataEx[9];
  data[3]=dataEx[10];
  data[4]=dataEx[11];
  data[5]=dataEx[12];
  lengthString=6;

  ck1=checksum1(data,lengthString);	//6. Checksum1
  ck2=checksum2(ck1);				//7. Checksum2

  if (ck1 != dataEx[5]) return -1;
  if (ck2 != dataEx[6]) return -1;

  speedy = ((dataEx[10]&0xFF)<<8) | dataEx[9];
  return speedy;

}

// move one servo with continous rotation
void HerkulexClass::moveSpeedOne(int servoID, int Goal, int pTime, int iLed)
{
  if (Goal > 1023 || Goal < -1023) return;              // speed (goal) non correct
  if ((pTime <0) || (pTime > 2856)) return;

  int GoalSpeedSign;
  if (Goal < 0) {
    GoalSpeedSign = (-1)* Goal ;
    GoalSpeedSign |= 0x4000;  //bit n°14 
  } 
  else {
    GoalSpeedSign = Goal;
  }
  int speedGoalLSB=GoalSpeedSign & 0X00FF; 		       // MSB speedGoal 
  int speedGoalMSB=(GoalSpeedSign & 0xFF00) >> 8;      // LSB speedGoal 

  //led 
  int iBlue=0;
  int iGreen=0;
  int iRed=0;
  switch (iLed) {
  case 1:
    iGreen=1;
    break;
  case 2:
    iBlue=1;
    break;
  case 3:
    iRed=1;
    break;
  }
  int SetValue=2+iGreen*4+iBlue*8+iRed*16;		//assign led value 

  playTime=int((float)pTime/11.2);				// 8. Execution time

  pSize = 0x0C;              					// 3.Packet size 7-58
  cmd   = HSJOG;      					        // 5. CMD

  data[0]=speedGoalLSB;            			    // 8. speedLSB
  data[1]=speedGoalMSB;              			// 9. speedMSB
  data[2]=SetValue;                          	// 10. Mode=0;
  data[3]=servoID;                    			// 11. ServoID

  pID=servoID^playTime;

  lengthString=4;             					// lengthData

  ck1=checksum1(data,lengthString);				//6. Checksum1
  ck2=checksum2(ck1);							//7. Checksum2

  pID=servoID;

  dataEx[0] = 0xFF;				// Packet Header
  dataEx[1] = 0xFF;				// Packet Header	
  dataEx[2] = pSize;	 		// Packet Size
  dataEx[3] = pID;				// Servo ID
  dataEx[4] = cmd;				// Command Ram Write
  dataEx[5] = ck1;				// Checksum 1
  dataEx[6] = ck2;				// Checksum 2
  dataEx[7] = playTime;  		// Execution time	
  dataEx[8] = data[0];
  dataEx[9] = data[1];
  dataEx[10] = data[2];
  dataEx[11] = data[3];
  
  sendData(dataEx, pSize);

}

// move one servo at goal position 0 - 1024
void HerkulexClass::moveOne(int servoID, int Goal, int pTime, int iLed)
{
  if (Goal > 1023 || Goal < 0) return;              // speed (goal) non correct
  if ((pTime <0) || (pTime > 2856)) return;

  // Position definition
  int posLSB=Goal & 0X00FF;								// MSB Pos
  int posMSB=(Goal & 0XFF00) >> 8;						// LSB Pos

  //led 
  int iBlue=0;
  int iGreen=0;
  int iRed=0;
  switch (iLed) {
  case 1:
    iGreen=1;
    break;
  case 2:
    iBlue=1;
    break;
  case 3:
    iRed=1;
    break;
  }
  int SetValue=iGreen*4+iBlue*8+iRed*16;	//assign led value 

  playTime=int((float)pTime/11.2);			// 8. Execution time

  pSize = 0x0C;          			    	// 3.Packet size 7-58
  cmd   = HSJOG;              				// 5. CMD

  data[0]=posLSB;               			// 8. speedLSB
  data[1]=posMSB;               			// 9. speedMSB
  data[2]=SetValue;                         // 10. Mode=0;
  data[3]=servoID;                    		// 11. ServoID

    pID=servoID^playTime;

  lengthString=4;             				// lengthData

  ck1=checksum1(data,lengthString);			//6. Checksum1
  ck2=checksum2(ck1);						//7. Checksum2

  pID=servoID;

  dataEx[0] = 0xFF;				// Packet Header
  dataEx[1] = 0xFF;				// Packet Header	
  dataEx[2] = pSize;	 		// Packet Size
  dataEx[3] = pID;				// Servo ID
  dataEx[4] = cmd;				// Command Ram Write
  dataEx[5] = ck1;				// Checksum 1
  dataEx[6] = ck2;				// Checksum 2
  dataEx[7] = playTime;  		// Execution time	
  dataEx[8] = data[0];
  dataEx[9] = data[1];
  dataEx[10] = data[2];
  dataEx[11] = data[3];

  sendData(dataEx, pSize);

}

// move one servo to an angle between -160 and 160
void HerkulexClass::moveOneAngle(int servoID, float angle, int pTime, int iLed) {
	if (angle > 160.0|| angle < -160.0) return;	
	int position = (int)(angle/0.325) + 512;
	moveOne(servoID, position, pTime, iLed);
}

// write registry in the RAM: one byte 
void HerkulexClass::writeRegistryRAM(int servoID, int address, int writeByte)
{
  pSize = 0x0A;               	// 3.Packet size 7-58
  pID   = servoID;     			// 4. Servo ID - 253=all servos
  cmd   = HRAMWRITE;          	// 5. CMD
  data[0]=address;              // 8. Address
  data[1]=0x01;               	// 9. Lenght
  data[2]=writeByte;            // 10. Write error=0
 
  lengthString=3;             	// lengthData

  ck1=checksum1(data,lengthString);	//6. Checksum1
  ck2=checksum2(ck1);				//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header	
  dataEx[2] = pSize;	 	// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 		// Address 52
  dataEx[8] = data[1]; 		// Length
  dataEx[9] = data[2]; 		// Value1
  dataEx[10]= data[3]; 		// Value2

  sendData(dataEx, pSize);

}

// write registry in the EEP memory (ROM): one byte 
void HerkulexClass::writeRegistryEEP(int servoID, int address, int writeByte)
{
  pSize = 0x0A;                  // 3.Packet size 7-58
  pID   = servoID;     	         // 4. Servo ID - 253=all servos
  cmd   = HEEPWRITE;             // 5. CMD
  data[0]=address;               // 8. Address
  data[1]=0x01;                  // 9. Lenght
  data[2]=writeByte;             // 10. Write error=0
 
  lengthString=3;           	 // lengthData

  ck1=checksum1(data,lengthString);	//6. Checksum1
  ck2=checksum2(ck1);				//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header	
  dataEx[2] = pSize;		// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 		// Address 52
  dataEx[8] = data[1]; 		// Length
  dataEx[9] = data[2]; 		// Value1
  dataEx[10]= data[3]; 		// Value2

  sendData(dataEx, pSize);

}



// Private Methods //////////////////////////////////////////////////////////////

// createPacket
// returns the size of the packet, fills argument char* packet with packet, can be used with sendData(int, int)
void HerkulexClass::createPacket(byte PID, byte CMD, byte * data)
{
      // brute force find length of input data array
      byte dataSize = 0;
      byte chksm1 = 0;
      byte chksm2 = 0;
      while(data[dataSize] != '\0')
      {
        dataSize++;
      }
      //Serial.println(dataSize); 
      // add 7 for minimum size of packet (bytes) with no data
      byte packetSize = dataSize + 7;

      // checksum logic, see p 20 Herkulex DRS 0101, 0202 User Manual
      chksm1 ^= packetSize;
      chksm1 ^= PID;
      chksm1 ^= CMD;

      for(int i=0; i<dataSize; i++)
      {
        chksm1 ^= data[i];
      }
      
      chksm2 = ~chksm1;
      chksm1 &= 0xFE;
      chksm2 &= 0xFE;

      byte packet[packetSize];

      packet[0] = 0xFF;
      packet[1] = 0xFF;
      packet[2] = packetSize;
      packet[3] = PID;
      packet[4] = CMD;
      packet[5] = chksm1;
      packet[6] = chksm2;
	
      char str[50];
      int n;
      for(int i=0; i<packetSize; i++)
      {
        packet[i+7] = data[i];
	if(packet[i]<16)Serial.print('0');
	Serial.print(packet[i],HEX);
	if(i%2!=0) Serial.print(' ');
      }
      Serial.println();
      //Serial.println(packet);
      sendData(packet, packetSize);
      
      clearBuffer();
}	

// add data to variable list servo for syncro execution
void HerkulexClass::addData(int GoalLSB, int GoalMSB, int set, int servoID)
{
  moveData[conta++]=GoalLSB;  
  moveData[conta++]=GoalMSB;
  moveData[conta++]=set;
  moveData[conta++]=servoID;
}

// Sending the buffer long length to Serial port
void HerkulexClass::sendData(byte* buffer, int length)
{
		clearBuffer(); 		//clear the serialport buffer - try to do it!
        switch (port)
		{
			case SSerial:
						SwSerial.write(buffer, length);
						delay(1);
						break;
			#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
			case HSerial1:
				Serial1.write(buffer, length);
				delay(1);
				break;
			case HSerial2:
				Serial2.write(buffer, length);
				delay(1);
				break;
			case HSerial3:
				Serial3.write(buffer, length);
				delay(1);
				break;
			#endif
		}
}

// * Receiving the length of bytes from Serial port
void HerkulexClass::readData(int size)
{
	int i = 0;
    int beginsave=0;
    int Time_Counter=0;
    switch (port)
	{
	case SSerial:

        while((SwSerial.available() < size) & (Time_Counter < TIME_OUT)){
        		Time_Counter++;
        		delayMicroseconds(1000);  //wait 1 millisecond for 10 times
		}
        	
		while (SwSerial.available() > 0){
			byte inchar = (byte)SwSerial.read();
			if ( (inchar == 0xFF) & ((byte)SwSerial.peek() == 0xFF) ){
					beginsave=1; 
					i=0; 				 // if found new header, begin again
			}
			if (beginsave==1 && i<size) {
				   dataEx[i] = inchar;
				   i++;
			}
		}
		SwSerial.flush();
		break;
	
	#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
	case HSerial1:
		while((Serial1.available() < size) & (Time_Counter < TIME_OUT)){
        		Time_Counter++;
        		delayMicroseconds(1000);
		}      	
		while (Serial1.available() > 0){
      		byte inchar = (byte)Serial1.read();
			//printHexByte(inchar);
        	if ( (inchar == 0xFF) & ((byte)Serial1.peek() == 0xFF) ){
						beginsave=1;
						i=0; 						
             }
            if (beginsave==1 && i<size) {
                       dataEx[i] = inchar;
                       i++;
			}
		}
		break;
	
	case HSerial2:
	    while((Serial2.available() < size) & (Time_Counter < TIME_OUT)){
        		Time_Counter++;
        		delayMicroseconds(1000);
		}
        	
		while (Serial2.available() > 0){
			byte inchar = (byte)Serial2.read();
			if ( (inchar == 0xFF) & ((byte)Serial2.peek() == 0xFF) ){
					beginsave=1;
					i=0; 					
			}
			if (beginsave==1 && i<size) {
				   dataEx[i] = inchar;
				   i++;
			}
		}
		break;

	case HSerial3:
		while((Serial3.available() < size) & (Time_Counter < TIME_OUT)){
			Time_Counter++;
			delayMicroseconds(1000);
		}
		
		while (Serial3.available() > 0){
			byte inchar = (byte)Serial3.read();
			if ( (inchar == 0xFF) & ((byte)Serial3.peek() == 0xFF) ){
					beginsave=1;
					i=0; 
			}
			if (beginsave==1 && i<size) {
				   dataEx[i] = inchar;
				   i++;
			}
		}
		break;
	#endif
	}
}

//clear buffer in the serial port - better - try to do this
void HerkulexClass::clearBuffer()
{
  switch (port)
	{
	case SSerial:
                SwSerial.flush();
                delay(1);
                break;
	#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
	case HSerial1:
				Serial1.flush();
				while (Serial1.available()){
				Serial1.read();
				delayMicroseconds(200);
				}

		break;
	case HSerial2:
	            Serial2.flush();
				while (Serial2.available()){
				Serial2.read();
				delayMicroseconds(200);
				}
		break;
	case HSerial3:
	            Serial3.flush();
				while (Serial3.available()){
					Serial3.read();
					delayMicroseconds(200);
				}

		break;
	#endif
	}
}

void HerkulexClass::printHexByte(byte x)
{
  Serial.print("0x");
  if (x < 16) {
    Serial.print('0');
  }
    Serial.print(x, HEX);
    Serial.print(" ");

}



 HerkulexClass Herkulex;
