#include <Arduino.h>
#include "lis2mdl.h"
#include <Wire.h>

lis2mdl::lis2mdl(){;}

uint8_t lis2mdl::get(uint8_t addr)
{
  //get value at given adrress use wire. functions
  Wire.beginTransmission(LIS2MDL_ADDR);
  Wire.write(addr);
  uint8_t val = Wire.read();
  Wire.endTransmission();
  return val;
}

void lis2mdl::set(uint8_t addr, uint8_t value){
  //set value at given address use wire. functions
  Wire.beginTransmission(LIS2MDL_ADDR);
  Wire.write(addr);
  Wire.write(value);
  Wire.endTransmission();
}

void lis2mdl::init(){
  
  this->set(LIS2MDL_CFG_A,0x0C);
  this->set(LIS2MDL_CFG_C,0x10);

}

void lis2mdl::calibrate(int n_seconds){
  
  mag_raw raw;

  for(int i=0;i<(100*n_seconds);i++){

    // Read all axes for raw values
    this->read_raw(&raw);

    // Track minimum read in each axis
    for (int j=0;j<3;j++){

      if (mag_calib[j] < (raw.a[j]+299)){
        mag_calib[j] = (raw.a[j]+299);
      }

    }

    delay(10);
    if (!(i%100)){
      SerialUSB.print('.');
    }
  }

}

void lis2mdl::calib_set(int16_t calib[]){

  for (int i=0; i<3;i++){

    mag_calib[i] = calib[i];

  }

}

void lis2mdl::calib_get(int16_t calib[]){

  for (int i=0; i<3;i++){

    calib[i] = mag_calib[i];
    
  }


}


void lis2mdl::read_raw(mag_raw *raw){
  uint8_t buf[6];
  Wire.beginTransmission(LIS2MDL_ADDR);
  Wire.write(LIS2MDL_OUT_START);
  Wire.endTransmission(false);

  Wire.requestFrom(LIS2MDL_ADDR,6); //returns number of bytes from slave device

  
  for(int i=0; i<6; i++)  {
    buf[i] = Wire.read();   
  }

  for(int i = 0; i<3; i++)  {
     raw->a[i] = (int16_t)((buf[(2*i)+1] << 8) | buf[2*i]);
  }
  
}

void lis2mdl::read_float(mag_float *flt, mag_raw *raw){
  
  uint8_t buf[6];
  Wire.beginTransmission(LIS2MDL_ADDR);
  Wire.write(LIS2MDL_OUT_START);
  Wire.endTransmission(false);

  Wire.requestFrom(LIS2MDL_ADDR,6); //returns number of bytes from slave device

  for(int i=0; i<6; i++)  {
    buf[i] = Wire.read();   
  }

  for(int i=0; i<3; i++)  {
    raw->a[i] = (int16_t)((buf[(2*i)+1] << 8) | buf[2*i]);
    flt->a[i] = (float)((raw->a[i])*LIS2MDL_CONV);
  }
  
}

void lis2mdl::read_float(mag_float *flt){
  
  uint8_t buf[6];
  Wire.beginTransmission(LIS2MDL_ADDR);
  Wire.requestFrom(LIS2MDL_OUT_START,6); //returns number of bytes from slave device

  Wire.endTransmission();

  for(int i=0; i<6; i++)  {
    buf[i] = Wire.read();   
  }

  for(int i=0; i<3; i++)  {
    flt->a[i] = (float)(((buf[(2*i)+1] << 8) | buf[2*i])*LIS2MDL_CONV);
  }
  
  
}
