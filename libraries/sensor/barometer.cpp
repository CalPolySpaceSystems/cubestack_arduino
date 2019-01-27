#include "barometer.h"
#include <Wire.h>
#include <math.h>

// Barometer Configuration
#define BARO_PRESSURE 	(0x48)
#define BARO_TEMP 		(0x58)
#define BARO_ADDRESS 	(0x77)
#define BARO_RESET 		(0x1E)
#define BARO_PROM_READ 	(0xA2)
#define BAROMETER_ID 4

// calibration data
uint16_t C[7] = {0};

// Initialize barometer from unknown state
void initMS5607(void) {
  // reset the barometer
  Wire.beginTransmission(BARO_ADDRESS);
  Wire.write(BARO_RESET);
  Wire.endTransmission();
  delay(100);
  
  digitalWrite(13, LOW);

  for(uint8_t i = 0; i < 6; i++) {
    // write into PROM data
    Wire.beginTransmission(BARO_ADDRESS);
    Wire.write(BARO_PROM_READ + (i * 2));
    Wire.endTransmission();

    // read calibration data
    Wire.requestFrom(BARO_ADDRESS, (uint8_t) 2);
    while(Wire.available() == 0);

    digitalWrite(13, HIGH);

    C[i+1] = (Wire.read() << 8) | Wire.read();
    String help = "C" + String((i+1));
    help += ": ";
    help += String(C[i+1]);
    help += "\n";
    Serial.println(help);
    delay(10);
  }

  return;
}

// Primes MS5607 to organize temperature data for reading
void primeTempMS5607(void) {
  Wire.beginTransmission(BARO_ADDRESS);
  Wire.write(BARO_TEMP);
  Wire.endTransmission();
  return;
}

// Primes MS5607 to orgainze pressure data for reading
void primePressureMS5607(void) {
  Wire.beginTransmission(BARO_ADDRESS);
  Wire.write(BARO_PRESSURE);
  Wire.endTransmission();
  return;
}

// Reads the raw values based on the previous priming method (Temp or Pressure)
uint32_t readRawMS5607(void) {
  Wire.beginTransmission(BARO_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(BARO_ADDRESS, 3);
  while(Wire.available() == 0)
  ;
  return (((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | (uint32_t)Wire.read());
}

/* Reads MS5607 temperature data into MS5607data struct
 * Assumptions:
 *     MS5607 primed for temperature
 *     10 ms delay after priming
 */
void readTempMS5607(struct BAROMETER_packet *data) {
  uint32_t D2;
  int32_t T;
  int32_t dT;

  D2  = readRawMS5607();
  //Serial.print("Temperature: ");
  //Serial.println(D2, HEX);
  dT = D2 - ((uint32_t)C[5] << 8);     //update '_dT'
  // Below, 'dT' and '_C[6]'' must be casted in order to prevent overflow
  // A bitwise division can not be dobe since it is unpredictible for signed integers
  T = 2000 + (((int64_t)dT * C[6])/8388608);
  data->temp = T / 100.0;
  return;
}

/* Reads MS5607 pressure data into MS5607data struct
 * Assumptions:
 *    MS5607 primed for pressure
 *    10 ms delay after priming
*/
void readPressureMS5607(struct BAROMETER_packet *data) {
  uint32_t D1 = readRawMS5607();
  //Serial.print("Pressure: ");
  //Serial.println(D1, HEX);

  int32_t dT = (int32_t)(((((data->temp) * 100.0) - 2000) * 8388608) / C[6]);


  int64_t OFF  = ((int64_t)C[2])*131072 + ((int64_t)C[4]*dT)/64;

  int64_t SENS = ((int64_t)C[1])*65536 + ((int64_t)C[3]*dT)/128;
  
  int32_t P = (D1*(SENS/2097152) - OFF)/32768;

  data->pressure = P / 100.0;

  return;
}

// Calculate altitude using data from MS5607data struct
void calcAltitudeMS5607(BAROMETER_packet *data) {
  data->altitude = ((pow((1013.25 / data->pressure), 1/5.257) - 1.0) * (data->temp + 273.15)) / 0.0065;
}

// Convert MS5607 struct data to string for output
String MS5607ToString(struct BAROMETER_packet *data) {
  String out = "Pressure = " + String(data->pressure);
  out += "\nTemperature = " + String(data->temp);
  out += "\nAltitude = " + String(data->altitude);
  out += "\n";
  return out;
}

// Add in RTC and ID to the packet for transmission
// void buildPacketMS5607(struct BAROMETER_packet *data) {
//     calcAltitudeMS5607(data);
//     data->rtc = 1000;
//     data->id = BAROMETER_ID;
//     return;
// }

