#ifndef lis2mdl_h
#define lis2mdl_h
#include "Arduino.h"
#include <Wire.h>

// from registar map on data sheet: http://www.st.com/content/ccc/resource/technical/document/datasheet/group3/29/13/d1/e0/9a/4d/4f/30/DM00395193/files/DM00395193.pdf/jcr:content/translations/en.DM00395193.pdf
#define LIS2MDL_OFFSET_X_REG_L        0x45 //R/W
#define LIS2MDL_OFFSET_X_REG_H        0x46 //R/W
#define LIS2MDL_OFFSET_Y_REG_L        0x47 //R/W
#define LIS2MDL_OFFSET_Y_REG_H        0x48 //R/W
#define LIS2MDL_OFFSET_Z_REG_L        0x49 //R/W
#define LIS2MDL_OFFSET_Z_REG_H        0x4A //R/W

#define LIS2MDL_WHO_AM_I              0x4F //R
#define LIS2MDL_CFG_A                 0x60 //R/W
#define LIS2MDL_CFG_B                 0x61 //R/W
#define LIS2MDL_CFG_C                 0x62 //R/W
#define LIS2MDL_INT_CTRL              0x63 //R/W
#define LIS2MDL_INT_SRC               0x64 //R
#define LIS2MDL_INT_THS_L             0x65 //R/W
#define LIS2MDL_INT_THS_H             0x66 //R/W
#define LIS2MDL_STATUS_REG            0x67 //R
#define LIS2MDL_OUT_START             0x68 //R
#define LIS2MDL_TEMP_OUT_START        0x6E //R

#define LIS2MDL_ADDR                   0x1E

//conversion factor
#define LIS2MDL_CONV .157 //Converts to uT

typedef union{ // for readraw

  struct {

        int16_t x;
        int16_t y;
        int16_t z;

  }__attribute__((packed));

  int16_t a[3];  
    
}mag_raw;

typedef union{ // for float

  struct {

        float x;
        float y;
        float z;
   
  }__attribute__((packed));

  float a[3];
  
} mag_float;


class lis2mdl
{
public:

    lis2mdl(); // This is the constructor and can have inputs
    uint8_t get(uint8_t addr);
    void set(uint8_t addr, uint8_t value);
    void init();
    void calibrate(int n_seconds);
    void calib_get(int16_t calib[]);
    void calib_set(int16_t calib[]);
    void read_raw(mag_raw *raw);  //(int16_t * destination);
    void read_float(mag_float *flt, mag_raw *raw);
    void read_float(mag_float *flt);

private:
  int16_t mag_calib[3];

};
#endif
