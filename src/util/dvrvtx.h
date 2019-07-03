
#include "arduino.h"
#include "HardwareSerial.h"

// Define tables of bands and power
#define BAND_A  1 
#define BAND_B  2
#define BAND_E  3
#define BAND_FS 4
#define BAND_R  5

#define PWR_25  0
#define PWR_200 1
#define PWR_500 2
#define PWR_800 3

class dvrvtx
{
    public:

        dvrvtx(HardwareSerial& serialInst, int recordPin);

        void setBandAndChannel(uint8_t band, uint8_t channel);

        void setPower(uint8_t power, int version);

        bool toggleRecord(bool force);

        //uint8_t calcCRC(uint8_t data, uint8_t len);

    private:

        uint8_t calcCRC8(uint8_t *data, uint8_t len);

        int recordToggle;
        uint32_t recordStart;
        HardwareSerial *SerialInst;

};