
#include "ptap_ads1115.h"
#include <Wire.h>
#include <math.h>

uint16_t raw_data_packet[6]; 
//format: [A at ch0, B at ch0, A at ch1, B at ch1, A at ch2, B at ch2]


void setup() {
  Serial.begin(9600);
  Wire.begin();  
}

void loop() {
  prime_ads1115(ADC_A_ADDR, 0); //A at ch0
  raw_data_packet[0] = readRaw_ads1115(ADC_A_ADDR);
  
  prime_ads1115(ADC_B_ADDR, 0); //B at ch0
  raw_data_packet[1] = readRaw_ads1115(ADC_B_ADDR);
  
  prime_ads1115(ADC_A_ADDR, 1); //A at ch1
  raw_data_packet[2] = readRaw_ads1115(ADC_A_ADDR);
  
  prime_ads1115(ADC_B_ADDR, 1); //B at ch1
  raw_data_packet[3] = readRaw_ads1115(ADC_B_ADDR);
  
  prime_ads1115(ADC_A_ADDR, 2); //A at ch2
  raw_data_packet[4] = readRaw_ads1115(ADC_A_ADDR);
  
  prime_ads1115(ADC_B_ADDR, 2); //B at ch2
  raw_data_packet[5] = readRaw_ads1115(ADC_B_ADDR);

  for(int i=0; i<=5; i++) {
  SerialUSB.println(raw_data_packet[i]);  
  }
  SerialUSB.println("-----");
  
  delay(500);
}
