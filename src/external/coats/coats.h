// cosmos.h

#include "arduino.h"
#include "HardwareSerial.h"

//#define WORD_8BIT

#define STATUS_ENABLE (1)
#define STATUS_DISABLE (0)

// Define checksum settings
#define NONE    0
#define XOR     1
#define CRC8    2

// Status
#define STATUS_ID 129
#define STAT_CMD_RX 0x31
#define STAT_INVALID 0x32
#define STAT_GARBLED 0x33

#define POLYGEN 0x07

typedef void (*cmdCallback)(uint8_t cmd);

typedef uint8_t (*checksumCallback)(uint8_t *data, uint8_t len);

class coats
{
  
	public:
	
		coats(uint16_t ending, bool statusEnable, uint8_t checkSum);

    /*
     *  Setup functions
     */ 
     
    void addTlm(uint8_t id, void *data, size_t dataSize);

    void addCmd(uint8_t id, cmdCallback callback);
     
    uint8_t buildTlm(uint8_t id, uint8_t packet[]);

    uint8_t buildTlm(uint8_t id, uint8_t packet[], uint32_t timeStamp);
  
    /*
     *  UART Interface
     */

    // Initialize a software or hardware serial connection
    void serialInit(HardwareSerial& serialInst,long baud);
    void serialInit(Serial_& serialInst, long baud);

    // Write a status packet to the serial interface
    void serialWriteStat(HardwareSerial& serialInst, uint8_t statMsg);
    void serialWriteStat(Serial_& serialInst, uint8_t statMsg);

    // Write a telemetry without a timestamp
    void serialWriteTlm(HardwareSerial& serialInst, uint8_t id);
    void serialWriteTlm(Serial_& serialInst, uint8_t id);

    // Write a telemetry packet with a given timestamp
    void serialWriteTlm(HardwareSerial& serialInst, uint8_t id, uint32_t timeStamp);
    void serialWriteTlm(Serial_& serialInst, uint8_t id, uint32_t timeStamp); 

    // Parse a command that comes in over the serial monitor
    void serialParseCmd(HardwareSerial& serialInst);
    void serialParseCmd(Serial_& serialInst);
				
	private:

    /* Interfaces */
    union{
      HardwareSerial *_hw;
      Serial_ *_sw;
    }SerialCOATS;
    

		/* You can cut the memory space used in an 8-bit processor by defining "WORD_8BIT"*/

		void 	*packetPointers[256];
    cmdCallback       cmdCallbackPtr[256];
		size_t 	packetSizes[256];
		uint16_t endString;
		checksumCallback  csCallbackPtr;
    bool statEnable;
    

};

 uint8_t calcXOR(uint8_t *data, uint8_t len);
 uint8_t calcCRC8(uint8_t *data, uint8_t len);
