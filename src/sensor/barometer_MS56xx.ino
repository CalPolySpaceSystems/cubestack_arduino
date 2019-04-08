//Test Sketch for the Barometer library

#include "barometer_MS56xx.h"
#include <Wire.h>
#include <math.h>

struct MS56xx_packet bar;

void setup() {
  Serial.begin(9600); //need to change this to SerialUSB.begin() if using a MKR ZERO
  Wire.begin();
  pinMode(13, OUTPUT);

}

void loop() {
  initMS56xx("MS5607"); //call the initialize function with the specified barometer model

  primeTempMS56xx();
  delay(10);

  readTempMS56xx(&bar);

  primePressureMS56xx();
  delay(10);

  readPressureMS56xx(&bar);

  calcAltitudeMS56xx(&bar);
  
  String output = MS56xxToString(&bar);
  Serial.println(output);
  delay(2000);
}
