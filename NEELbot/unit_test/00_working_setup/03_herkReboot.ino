void herkReboot(byte servoID) {
  // put your setup code here, to run once:
  byte testData[1] = {'\0'};
  createPacket(servoID, 0x09, testData);
}


