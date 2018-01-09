

char testData[3] = {'\0'};
byte testPacket[223] = {};




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  createPacket(19, 9, testData, testPacket);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void createPacket(byte PID, byte CMD, byte * data, byte * packet)
{
      // brute force find length of input data array
      byte dataSize = 0;
      byte chksm1 = 0;
      byte chksm2 = 0;
      while(data[dataSize] != '\0')
      {
        //Serial.println((int)data[dataSize]);
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

      packet[0] = 0xFF;
      packet[1] = 0xFF;
      packet[2] = packetSize;
      packet[3] = PID;
      packet[4] = CMD;
      packet[5] = chksm1;
      packet[6] = chksm2;

      for(int i=0; i<packetSize; i++)
      {
        packet[i+7] = data[i];
      }

      for(int i=0; i<packetSize; i++)
      {
        Serial.write(packet[i]);
      }
}

