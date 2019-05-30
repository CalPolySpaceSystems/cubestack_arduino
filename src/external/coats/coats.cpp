// coats.cpp

#include "coats.h"

coats::coats(uint16_t ending, bool statusEnable, uint8_t checkSum)
{
	endString = ending;
  statEnable = statusEnable;

  switch (checkSum){

    case XOR:
      csCallbackPtr = &calcXOR;
      break;

    case CRC8:
      csCallbackPtr = &calcCRC8;
      break;
      
    default:
      csCallbackPtr = 0;
    
  }
 
}

/*
 *	UART Interface
 */

void coats::serialInit(HardwareSerial& serialInst, long baud)
{
	SerialCOATS._hw = (HardwareSerial*)&serialInst;
	SerialCOATS._hw->begin(baud);
	
}

void coats::serialInit(Serial_& serialInst, long baud)
{
  SerialCOATS._sw = (Serial_*)&serialInst;
  SerialCOATS._sw->begin(baud);
  
}

/* Send a telemetry packet */
void coats::serialWriteTlm(HardwareSerial& serialInst, uint8_t id){
  
  size_t sz = packetSizes[id];
  uint8_t *pData = (uint8_t*)packetPointers[id];
  uint8_t checkSum = 0;

  SerialCOATS._hw->write(id);
  SerialCOATS._hw->write(pData, sz);
  
  if (csCallbackPtr){ SerialCOATS._hw->write(csCallbackPtr(pData,sz));}

  SerialCOATS._hw->write(endString >> 8);
  SerialCOATS._hw->write(endString); 
  
}

void coats::serialWriteTlm(Serial_& serialInst, uint8_t id){
  
  size_t sz = packetSizes[id];
  uint8_t *pData = (uint8_t*)packetPointers[id];
  uint8_t checkSum = 0;

  SerialCOATS._sw->write(id);
  SerialCOATS._sw->write(pData, sz);
  
  if (csCallbackPtr){ SerialCOATS._sw->write(csCallbackPtr(pData,sz));}

	SerialCOATS._sw->write(endString >> 8);
  SerialCOATS._sw->write(endString); 

}

void coats::serialWriteTlm(HardwareSerial& serialInst, uint8_t id, uint32_t timeStamp){
  
  size_t sz = packetSizes[id];
  uint8_t *pData = (uint8_t*)packetPointers[id];
  uint8_t checkSum = 0;

  SerialCOATS._hw->write(id);
  SerialCOATS._hw->write(timeStamp);
  SerialCOATS._hw->write(timeStamp >> 8);
  SerialCOATS._hw->write(timeStamp >> 16);
  SerialCOATS._hw->write(timeStamp >> 24);
  SerialCOATS._hw->write(pData, sz);

  if (csCallbackPtr){ SerialCOATS._hw->write(csCallbackPtr(pData,sz));}

  SerialCOATS._hw->write(endString >> 8);
  SerialCOATS._hw->write(endString); 

}

void coats::serialWriteTlm(Serial_& serialInst, uint8_t id, uint32_t timeStamp){
  
  size_t sz = packetSizes[id];
  uint8_t *pData = (uint8_t*)packetPointers[id];
  uint8_t checkSum = 0;
  
  SerialCOATS._sw->write(id);
  SerialCOATS._sw->write(timeStamp);
  SerialCOATS._sw->write(timeStamp >> 8);
  SerialCOATS._sw->write(timeStamp >> 16);
  SerialCOATS._sw->write(timeStamp >> 24);
  SerialCOATS._hw->write(pData, sz);

  if (csCallbackPtr){ 
    SerialCOATS._sw->write(csCallbackPtr(pData,sz));
  }

  SerialCOATS._sw->write(endString >> 8);
  SerialCOATS._sw->write(endString); 

}


/* Send a Status Packet */
void coats::serialWriteStat(HardwareSerial& serialInst, uint8_t statMsg){

  SerialCOATS._hw->write(STATUS_ID);
  SerialCOATS._hw->write(statMsg);
  SerialCOATS._hw->write(endString >> 8);
  SerialCOATS._hw->write(endString); 

}

void coats::serialWriteStat(Serial_& serialInst, uint8_t statMsg){

  SerialCOATS._sw->write(STATUS_ID);
  SerialCOATS._sw->write(statMsg);
  SerialCOATS._sw->write(endString >> 8);
  SerialCOATS._sw->write(endString); 

}

/* Reads a command from the serial port given
 * 
 * FORMAT: 
 * | CMD | CMD Value | Checksum | 
 */

void coats::serialParseCmd(HardwareSerial& serialInst){

  // Check for input
  if (SerialCOATS._hw->available()){
    
    uint8_t input[3];

    // Read from buffer
    SerialCOATS._hw->readBytes(input,3);

    // Flush everything else from the buffer.
    while (SerialCOATS._hw->available()){
      char c = SerialCOATS._hw->read();
    }

    // Read command checksum (CRC8)
    uint8_t cs_calc = calcCRC8((uint8_t*)input,2);
    
    if(input[2]==cs_calc){

      // Send status
      this->serialWriteStat(serialInst, STAT_CMD_RX);

      // Send parameter to callback function
      cmdCallback cb = cmdCallbackPtr[input[0]];

      // Send parameter to callback function
      cb(input[1]);
      
    }

    else{

      // Inform of packet garbled
      this->serialWriteStat(serialInst, STAT_GARBLED);
        
    }
      
  }
  
}
  
void coats::serialParseCmd(Serial_& serialInst){

  // Check for input
  if (SerialCOATS._sw->available()){
    
    char input[3];

    // Read from buffer
    SerialCOATS._sw->readBytes(input,3);
    
    // Flush everything else from the buffer.
    while (SerialCOATS._sw->available()){
      char c = SerialCOATS._sw->read();
    }

    // Read command checksum (CRC8)
    uint8_t cs_calc = calcCRC8((uint8_t*)input,2);

    if(input[2]==cs_calc){
      
      // Send status
      this->serialWriteStat(serialInst, STAT_CMD_RX);

      cmdCallback cb = cmdCallbackPtr[input[0]];
      
      // Send parameter to callback function
      cb(input[1]);
        
    }

    else{

      // Inform of packet garbled
      this->serialWriteStat(serialInst, STAT_GARBLED);
       
    }
      
  }

}
  
/* Adds a new telemetry packet */ 
void coats::addTlm(uint8_t id, void *data, size_t dataSize){
	
	packetSizes[id] = dataSize;
	packetPointers[id] = data;

}

/* Adds a command */
void coats::addCmd(uint8_t id, cmdCallback callback){
  cmdCallbackPtr[id] = callback;
}


/* Outputs a packet as a string */
uint8_t coats::buildTlm(uint8_t id, uint8_t *packet){

  size_t sz = packetSizes[id];
  uint8_t *pData = (uint8_t*)packetPointers[id];
  uint8_t checkSum = 0;

  packet[0] = id;

  for (uint8_t i=0;i<sz;i++)
  {
    packet[i+1] = *(pData+i);
  }

  if (csCallbackPtr){ 
    packet[sz+1] = csCallbackPtr(pData,sz);
    sz+=1;
  }

  packet[sz+1] = (endString >> 8);
  packet[sz+2] = (endString);

  return (sz+3);

}

uint8_t coats::buildTlm(uint8_t id, uint8_t *packet, uint32_t timeStamp){

  size_t sz = packetSizes[id];
  uint8_t *pData = (uint8_t*)packetPointers[id];
  uint8_t checkSum = 0;

  packet[0] = id;
  packet[1] = timeStamp;
  packet[2] = (timeStamp >> 8);
  packet[3] = (timeStamp >> 16);
  packet[4] = (timeStamp >> 24);

  for (uint8_t i=0;i<sz;i++)
  {
    packet[i+5] = *(pData+i);
  }

  if (csCallbackPtr){ 
    packet[sz+5] = csCallbackPtr(pData,sz);
    sz+=1;
  }

  packet[sz+5] = (endString >> 8);
  packet[sz+6] = (endString);

  return (sz+7);

}

/* Checksum Calculations */

uint8_t calcXOR(uint8_t *data, uint8_t len){

  uint8_t cs = 0;
  
  for (uint8_t i=0;i<len;i++){

        cs ^= *(data+i);

  }

  return cs;
  
}

uint8_t calcCRC8(uint8_t *data, uint8_t len){
  
    uint8_t crc = 0x00;
    
    while (len--) {
      
      uint8_t extract = *data++;
      
      for (uint8_t tempI = 8; tempI; tempI--) {
        uint8_t sum = (crc ^ extract) & 0x01;
        
        crc >>= 1;
        
        if (sum) {
          crc ^= 0x8C;
        }
        
        extract >>= 1;
      }
    }
    
    return crc;

}
