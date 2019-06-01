//Library for the Pressure Tap Board
//board contains 2 ADCs with part number ADS1115
// 5/2019

#ifndef ads1115_h
#define ads1115_h

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define ADC_A_ADDR  0x48 //ADC device addresses
#define ADC_B_ADDR  0x49 //ADC device addresses

#define POINT_REG_CONV  0b00000000
#define POINT_REG_CONFIG  0b00000001
//bit 1 and 0 in the Address Pointer Register determine what register is being accessed,
//so 00000000 is for the conversion register and 00000001 is for the configuration 
//register.

//the configuration register is 16 bit so it needs 2 bytes
#define BLANK_INIT_CONFIG 0b01000001 //enters power down state
#define INIT_CONFIG2 0b11100011 //sets to 860 samples per second

#define AIN0_CONFIG1 0b11000001 //this is for channel 0 or AIN0

#define AIN1_CONFIG1 0b11010001 //channel 1 or AIN1

#define AIN2_CONFIG1 0b11100001 //channel 2 or AIN2

#define AINX_CONFIG2 0b11100011 //the second byte is unchanged for all 3 channels

#define BLANK_CHANNEL_CONFIG 0b11000001 


void writeReg(byte deviceAddress, byte targetRegister, byte newValue);

uint16_t readReg(byte deviceAddress, byte addr);

void prime_ads1115(byte deviceAddress, int channel);

uint16_t readRaw_ads1115(byte deviceAddress);


#endif
