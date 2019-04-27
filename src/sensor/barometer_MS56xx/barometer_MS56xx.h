#ifndef _BARO_H
#define _BARO_H
#include <Arduino.h>

//This library should be compatible with both the MS5611 and the MS5607

//Note: all data reads must be primed and allow for a 10ms delay
//after priming before reading.

struct MS56xx_packet { //This will be the data in the correct packet
  //"format" for COSMOS
  float temp;
  float pressure;
  float altitude;
}__attribute__((packed));

void initMS56xx(String); 

//Temperature functions must all primeTemp before readTemp
void primeTempMS56xx(void); 
void readTempMS56xx(struct MS56xx_packet *data); 


//Pressure functions must call primePressure befrore readPressure
void primePressureMS56xx(void);
void readPressureMS56xx(struct MS56xx_packet *data);

//Calculate altitude using data from MS5611_packet struct
void calcAltitudeMS56xx(struct MS56xx_packet *data);
//calcAlt..(&variable)

//convert MS5611 struct data to string for output
String MS56xxToString(struct MS56xx_packet *data);

/*The function below reads the raw values based on the previous 
 * priming method (Temp or Pressure)
 * ONLY CALL THIS IF YOU MUST!
 * IT IS ALREADY IMPLEMENTED IN readTempMS6511 and readPressureMS5611
*/
uint32_t readRawMS56xx();
//uint32_t means this is an unsigned integer that has a size of
//32 bits, and the size is standard across all platforms

#endif
