/* Cal Poly Space Systems
 * 
 * imu_triple.cpp 
 *
 */

#include "imu_triple.h"
#include <SPI.h>

/* Initialization of class */
imu_triple::imu_triple(int cs_fine, int cs_coarse, int cs_hi_g){

    /* Save chip select values */
    cs_f = cs_fine;
    cs_c = cs_coarse;
    cs_h = cs_hi_g;
    
}

/* Access internal addresses */
void imu_triple::init(){
  
  /* Init fine sensor */
  this->set(LSM6DSL_FINE,LSM6DSL_CTRL1_XL,0x70);
  this->set(LSM6DSL_FINE,LSM6DSL_CTRL2_G,0x70);

  /* Init coarse sensor */
  this->set(LSM6DSL_COARSE,LSM6DSL_CTRL1_XL,0x74);
  this->set(LSM6DSL_COARSE,LSM6DSL_CTRL2_G,0x7C);

  /* Init hi-g sensor */
  this->set(H3LIS331DL,H3LIS331DL_CTRL1,0x3F);
  this->set(H3LIS331DL,H3LIS331DL_CTRL4,0x00);


}

/* Access internal addresses */
void imu_triple::set(imu_device_t dev, uint8_t addr, uint8_t value){
    
    int cs_pin;
    
    /* Decide which device to read from */
    if (dev == LSM6DSL_FINE){
        cs_pin = cs_f;
    }
    
    else if(dev == LSM6DSL_COARSE){
        cs_pin = cs_c;
    }

    else{
        cs_pin = cs_h;
    }

    /* Write to the desired sensor */
    SPI.beginTransaction(SPISettings(SPI_SPEED,SPI_ENDIAN,SPI_MODE));
    
    digitalWrite(cs_pin, LOW);
    SPI.transfer(addr);
    SPI.transfer(value);
    digitalWrite(cs_pin, HIGH);

    SPI.endTransaction();

}

uint8_t imu_triple::get(imu_device_t dev, uint8_t addr){

    int cs_pin;
    
    /* Decide which device to read from */
    if (dev == LSM6DSL_FINE){
        cs_pin = cs_f;
    }
    
    else if(dev == LSM6DSL_COARSE){
        cs_pin = cs_c;
    }

    else{
        cs_pin = cs_h;
    }

    /* Read from the desired sensor */
    SPI.beginTransaction(SPISettings(SPI_SPEED,SPI_ENDIAN,SPI_MODE));
    
    digitalWrite(cs_pin, LOW);
    uint8_t val = SPI.transfer(0x80 | addr);
    digitalWrite(cs_pin, HIGH);

    SPI.endTransaction();

    return val;

}

void imu_triple::calibrate(int n_samples){

    imu_raw raw;

    // Read each sensor and store its 
    for (int i=0;i<3;i++){

        for (int j=0; j<n_samples; j++){
            this->read_dev((imu_device_t)i,&raw);
            
            switch (i){

                case 0: // LSM6DSL Fine
                    for (int k=0;k<5;k++){
                        calib_fine[k] += (int16_t)(round(raw.a[k]/n_samples));
                    }
                    calib_coarse[5] += (int16_t)(round((raw.a[5]-16384)/n_samples));
                    break;

                case 1: // LSM6DSL Coarse

                    for (int k=0;k<5;k++){
                        calib_coarse[k] += (int16_t)(round(raw.a[k]/n_samples));
                    }
                    calib_coarse[5] += (int16_t)(round((raw.a[5]-2048)/n_samples));
                    break;

                case 2: // H3LISS31DL

                    for (int k=3;k<5;k++){
                        calib_hi_g[k] += (int16_t)(round(raw.a[k]/n_samples));
                    }
                    calib_hi_g[5] += (int16_t)(round((raw.a[5]-328)/n_samples));
                    break;

            }
        
            delay(10);

        }

    }

}

void imu_triple::calib_set(imu_device_t dev, int16_t calib[]){
    
    /* Decide which device to set calibration for */
    if (dev == LSM6DSL_FINE){
        
        for (int i=0; i<6;i++){

            calib_fine[i] = calib[i];

        }
    }
    
    else if(dev == LSM6DSL_COARSE){
        
        for (int i=0; i<6;i++){

            calib_coarse[i] = calib[i];

        }

    }

    else{
        for (int i=0; i<3;i++){

            calib_hi_g[i] = calib[i];

        }
    }

}

void imu_triple::calib_get(imu_device_t dev, int16_t calib[]){
    
    /* Decide which device to get calibration from */
    if (dev == LSM6DSL_FINE){
        
        for (int i=0; i<6;i++){
            calib[i] = calib_fine[i];
        }
    }
    
    else if(dev == LSM6DSL_COARSE){
        
        for (int i=0; i<6;i++){

            calib[i] = calib_coarse[i];

        }

    }

    else{
        for (int i=0; i<3;i++){

            calib[i] = calib_hi_g[i];

        }
    }

}

/* Reads raw values from the given device */
void imu_triple::read_dev(imu_device_t dev, imu_raw *raw){

    uint8_t buf[12];
    uint8_t acc_ofs = 0;
    int cs_pin;
    uint8_t regs;
    byte out_start;
    
    /* Read the fine A/G */
    if (dev == LSM6DSL_FINE){
        regs = 12;
        cs_pin = cs_f;
        out_start = LSM6DSL_OUT_START;
    }
    
    /* Read the coarse A/G */
    else if(dev == LSM6DSL_COARSE){
        regs = 12;
        cs_pin = cs_c;
        out_start = LSM6DSL_OUT_START;
    }

    /* Read the high-g accelerometer */
    else{
        regs = 6;
        cs_pin = cs_h;
        out_start = H3LIS331DL_OUT_START;
        acc_ofs = 3;
    }

    /* Fill buffer with all addresses to read from */
    for (uint8_t i=0;i<regs;i++){
        buf[i] = out_start+i+1;
    }

    /* Read the desired sensor */
    SPI.beginTransaction(SPISettings(SPI_SPEED,SPI_ENDIAN,SPI_MODE));
    
    digitalWrite(cs_pin, LOW);
    SPI.transfer(out_start);
    SPI.transfer(buf,regs);
    digitalWrite(cs_pin, HIGH);

    SPI.endTransaction();

    for (uint8_t i=0;i<regs;i=i+2){
        
        /* Put buffer into raw data array */
        raw->a[(i/2)+acc_ofs] = (buf[i] | (buf[i+1]<<8));

    }


}

/* Read data */ 
uint16_t imu_triple::read_raw(imu_raw *raw){
    
    uint8_t buf[12];
    uint16_t saturate = 0;

    /* Fill buffer with all addresses to read from */
    for (uint8_t i=0;i<12;i++){
        buf[i] = LSM6DSL_OUT_START+i+1;
    }

    /* Read the fine A/G */
    SPI.beginTransaction(SPISettings(SPI_SPEED,SPI_ENDIAN,SPI_MODE));
    
    digitalWrite(cs_f, LOW);
    SPI.transfer(LSM6DSL_OUT_START);
    SPI.transfer(buf,12);
    digitalWrite(cs_f, HIGH);

    SPI.endTransaction();

    for (uint8_t i=0;i<12;i=i+2){
        
        /* Put buffer into raw data array */
        raw->a[i/2] = (buf[i] | (buf[i+1]<<8));

        /* Find if any values are saturated */
        if (abs(raw->a[i/2]) > 32000){
            saturate |= (1<<(i/2));
        }
        else{
            raw->a[i/2] -= calib_fine[i/2];
        }
    
    }

    //SerialUSB.println(saturate);

    /* If necessary, read from the coarse A/G */
    if (saturate){

        //SerialUSB.println("COARSE");

        uint8_t ind_new;

        /* Fill buffer with all addresses to read from */
        for (uint8_t i=0;i<12;i++){
            buf[i] = LSM6DSL_OUT_START+i+1;
        }

        /* Read from the coarse A/G */
        SPI.beginTransaction(SPISettings(SPI_SPEED,SPI_ENDIAN,SPI_MODE));
    
        digitalWrite(cs_c, LOW);
        SPI.transfer(LSM6DSL_OUT_START);
        SPI.transfer(buf,12);
        digitalWrite(cs_c, HIGH);

        SPI.endTransaction();


        /* Write new data into the array of raw values only if they are saturated*/
        for (uint8_t i=0;i<12;i=i+2){
    
            if ((saturate) & (1<<(i/2))){

                //SerialUSB.println(i);
              
                raw->a[i/2] = (buf[i] | (buf[i+1]<<8));

                /* Find if accelerometer values are saturated */
                if ((abs(raw->a[i/2] > 32000))&(i>5)){
                    saturate |= (1<<((i/2)+3));
                }
                else{
                    raw->a[i/2] -= calib_coarse[i/2];
                }
                
            }

        }

    }

    //saturate = 0b111000000;

    /* If necessary, read from the high-g sensor */
    if (saturate & 0b111000000){

        //SerialUSB.println("HI G");

        /* Fill buffer with all addresses to read from */
        for (uint8_t i=0;i<6;i++){
            buf[i] = H3LIS331DL_OUT_START+i+1;
            //SerialUSB.println(buf[i],HEX);
        }

        /* Retieve data from hi-g sensor */
        SPI.beginTransaction(SPISettings(10000000,SPI_ENDIAN,SPI_MODE));
    
        digitalWrite(cs_h, LOW);
        SPI.transfer(H3LIS331DL_OUT_START);
        SPI.transfer(buf,6);
        digitalWrite(cs_h, HIGH);

        SPI.endTransaction();

        /* Put the hi-g values into the raw data array if necessary */
        for (uint8_t i=0;i<6;i=i+2){
    
            if (saturate & (1<<((i/2)+6))){

                //SerialUSB.println(buf[i]);
                
                // add calib
                raw->a[(i/2)+3] = (buf[i] | (buf[i+1]<<8)) - calib_hi_g[i/2];

            }

        }

        //SerialUSB.println(raw->a[3]);
        
    }

      return saturate;
}


void imu_triple::read_float(imu_float *flt){

    imu_raw raw;
    uint16_t saturate;

    /* Get raw data and saturation bitfield */
    saturate = this->read_raw(&raw);

    /* Convert data from the gyroscopes */
    for(int i=0;i<3;i++){

        if((1<<i)&saturate){
            flt->a[i] = raw.a[i]*LSM6DSL_C_GYR_CONV;
        }
        else{
            flt->a[i] = raw.a[i]*LSM6DSL_F_GYR_CONV;
        }

    }

    /* Convert the data from the accelerometer */
    for(int i=0;i<3;i++){

        if((1<<(i+3))&saturate){

            if((1<<(i+6))&saturate){
                //SerialUSB.println("High Acc");
                flt->a[i] = raw.a[i]*H3LIS331DL_ACC_CONV;
            }
            else{
                //SerialUSB.println("COARSE ADJ");
                flt->a[i] = raw.a[i]*LSM6DSL_C_ACC_CONV;
            }

        }
        else{
            flt->a[i] = raw.a[i]*LSM6DSL_F_ACC_CONV;
        }

    }

}

uint16_t imu_triple::read_float(imu_float *flt, imu_raw *raw){

    /* Get raw data and saturation bitfield */
    uint16_t saturate = this->read_raw(raw);


    /* Convert data from the gyroscopes */
    for(int i=0;i<3;i++){

        if((1<<i)&saturate){
            flt->a[i] = raw->a[i]*LSM6DSL_C_GYR_CONV;
        }
        else{
            flt->a[i] = raw->a[i]*LSM6DSL_F_GYR_CONV;
        }

    }

    /* Convert the data from the accelerometer */
    for(int i=0;i<3;i++){

        if((1<<(i+3))&saturate){

            if((1<<(i+6))&saturate){
                flt->a[i+3] = raw->a[i+3]*H3LIS331DL_ACC_CONV;
            }
            else{
                flt->a[i+3] = raw->a[i+3]*LSM6DSL_C_ACC_CONV;
            }

        }
        else{
            flt->a[i+3] = raw->a[i+3]*LSM6DSL_F_ACC_CONV;
        }

    }

  return saturate;
}