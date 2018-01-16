void createPacket(byte PID, byte CMD, byte * data)
{
      // brute force find length of input data array
      byte dataSize = 0;
      byte chksm1 = 0;
      byte chksm2 = 0;
      while(data[dataSize] != '\0')
      {
        dataSize++;
      }
      Serial.println(dataSize);
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

      for(int i=0; i<packetSize; i++)
      {
        packet[i+7] = data[i];
        if(packet[i]<16)Serial.print('0');
        Serial.print(packet[i],HEX);
        if(i%2!=0) Serial.print(' ');
      }
      Serial.println();
      
      SSerial.write(packet, packetSize);
      delay(1);
}

