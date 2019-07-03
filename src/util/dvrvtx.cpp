
// Define commands

#include "dvrvtx.h"

#define CMD_SET_POWER 0x05
#define CMD_SET_CHAN 0x07
#define CMD_SET_FREQ 0x09

#define POLYGEN 0xD5

dvrvtx::dvrvtx(HardwareSerial& serialVTX, int recordPin){

    // Set record pin as output
    pinMode(recordPin,OUTPUT);
    digitalWrite(recordPin,HIGH);

    //SerialInst = &serialVTX;

    // Begin serial connection to dvrvtx
    //SerialInst->begin(4800);

    // Set power to pit mode
    //this->setPower(PWR_25,1);
    
}

void dvrvtx::setBandAndChannel(uint8_t band, uint8_t channel){

    uint8_t buf[6] = {0xAA, 0x55, CMD_SET_CHAN ,1 } ;

    buf[4] = (band<<3) + channel;

    buf[5] = this->calcCRC8(buf, 5);

    // Write Command
    SerialInst->write(buf,6);


}

void dvrvtx::setPower(uint8_t power, int version){
    
    uint8_t buf[6] = {0xAA, 0x55, CMD_SET_POWER ,1 } ;

    if(version == 1){
        buf[4] = 7+(9*power);
    }
    else{
        buf[4] = power;
    }
    buf[5] = this->calcCRC8(buf, 5);

    SerialInst->write(buf,6);


}

uint8_t dvrvtx::calcCRC8(uint8_t *data, uint8_t len){

    uint8_t crc = 0;

    for (uint8_t i=0;i<len;i++){

        crc ^= data[i];

        for (uint8_t j=0;j<8;j++){
            if (crc & 0x80){
                crc = (uint8_t)((crc <<1) ^ POLYGEN);
            }
            else {
                crc << 1;
            }


        }

    }

    return crc;

}