void set_led(byte servoID, byte color) {

  byte testData[4] = {0x35, 0x01, color, '\0'};
  createPacket(servoID, 0x03, testData);
}



