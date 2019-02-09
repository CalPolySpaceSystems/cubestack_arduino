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

    /* Init fine sensor */
    imu_triple.set(LSM6DSL_FINE,LSM6DSL_CTRL1_XL,0x70);
    imu_triple.set(LSM6DSL_FINE,LSM6DSL_CTRL2_G,0x70);

    /* Init coarse sensor */
    imu_triple.set(LSM6DSL_COARSE,LSM6DSL_CTRL1_XL,0x7C);
    imu_triple.set(LSM6DSL_COARSE,LSM6DSL_CTRL2_G,0x7C);

    /* Init hi-g sensor */
    imu_triple.set(H3LIS331DL,H3LIS331DL_CTRL1,0x37);

}

/* TODOS
*   
*   set/get
*   read devices
*   set calibration, define calib array 
*   apply centripetal accel to float values
*
*/


/* Access internal addresses */
void imu_triple::set(imu_device_t dev, uint8_t addr, uint8_t value){
    
    /* Decide which device to read from */
    if (dev == LSM6DSL_FINE){
        int cs_pin = cs_f;
    }
    
    else if(dev == LSM6DSL_COARSE){
        int cs_pin = cs_c;
    }

    else{
        int cs_pin = cs_h;
    }

    /* Write to the desired sensor */
    digitalWrite(cs_pin, LOW);
    SPI.transfer16((addr<<8) | (value));
    digitalWrite(cs_pin, HIGH);

}

uint8_t imu_triple::get(imu_device_t dev, uint8_t addr){
    
    /* Decide which device to read from */
    if (dev == LSM6DSL_FINE){
        int cs_pin = cs_f;
    }
    
    else if(dev == LSM6DSL_COARSE){
        int cs_pin = cs_c;
    }

    else{
        int cs_pin = cs_h;
    }

    /* Read from the desired sensor */
    digitalWrite(cs_pin, LOW);
    SPI.transfer(buf,regs);
    digitalWrite(cs_pin, HIGH);

}

/* Reads raw values from the given device */
void imu_triple::read_dev(imu_device_t dev, imu_raw *raw){

    uint8_t buf[12];
    uint8_t acc_ofs = 0;

    /* Decide which device to read from */
    if (dev == LSM6DSL_FINE){
        uint8_t regs = 12;
        int cs_pin = cs_f;
        byte out_start = LSM6DSL_OUT_START;
    }
    
    else if(dev == LSM6DSL_COARSE){
        uint8_t regs = 12;
        int cs_pin = cs_c;
        byte out_start = LSM6DSL_OUT_START;
    }

    else{
        uint8_t regs = 6;
        int cs_pin = cs_h;
        byte out_start = H3LIS331DL_OUT_START;
        acc_ofs = 3;
    }

    /* Fill buffer with all addresses to read from */
    for (uint8_t i=0;i<regs;i++){
        buf[i] = out_start+i;
    }

    /* Read the desired sensor */
    digitalWrite(cs_pin, LOW);
    SPI.transfer(buf,regs);
    digitalWrite(cs_pin, HIGH);

    for (uint8_t i=0;i<regs;i=i+2){
        
        /* Put buffer into raw data array */
        raw->a[(i/2)+acc_ofs] = ((buf[i] << 8) | buf[i+1]);

    }


}

/* Set the calibration offsets for the sensor */
void imu_triple::set_calib(imu_device_t dev, int16_t *calib){
    // sets the declared calibration array
    
    // Set internal offset registers - A/G, accel, fine/coarse

}

/* Read data */ 
void imu_triple::read_raw(imu_raw *raw, uint16_t saturate){
    
    uint8_t buf[12];
    saturate = 0;

    /* Fill buffer with all addresses to read from */
    for (uint8_t i=0;i<12;i++){
        buf[i] = LSM6DSL_OUT_START+i;
    }

    /* Read the fine A/G */
    digitalWrite(cs_f, LOW);
    SPI.transfer(buf,12);
    digitalWrite(cs_f, HIGH);

    for (uint8_t i=0;i<12;i=i+2){
        
        /* Put buffer into raw data array */
        raw->a[i/2] = ((buf[i] << 8) | buf[i+1]);

        /* Find if any values are saturated */
        if ((raw->a[i/2] & 0x8F) > 0x8C){
            saturate |= (1<<(i/2));
        }
        else{
            // add calib
        }
    }

    /* If necessary, read from the coarse A/G */
    if (saturate){

        uint8_t ind_new;

        /* Fill buffer with all addresses to read from */
        for (uint8_t i=0;i<12;i++){
            buf[i] = LSM6DSL_OUT_START+i;
        }

        /* Read from the coarse A/G */
        digitalWrite(cs_c, LOW);
        SPI.transfer(buf,12);
        digitalWrite(cs_c, HIGH);

        /* Write new data into the array of raw values only if they are saturated*/
        for (uint8_t i=0;i<12);i=i+2){
    
            if (saturate & (1<<i)){

             
                raw->a[i/2] = ((buf[i] << 8) | buf[i+1]);

                /* Find if accelerometer values are saturated */
                if (((raw->a[i/2] & 0x8F) > 0x8C)&(i>4)){
                    saturate |= (1<<((i/2)+6));
                }
                else{
                    raw->a[i/2] += calib[(i/2)+6]
                }
                
            }

        }

    }

    /* If necessary, read from the high-g sensor */
    if (saturate & 0b111000000){

        /* Fill buffer with all addresses to read from */
        for (uint8_t i=0;i<6;i++){
            buf[i] = H3LIS331DL_OUT_START+i;
        }

        /* Retieve data from hi-g sensor */
        digitalWrite(cs_h, LOW);
        SPI.transfer(buf,6);
        digitalWrite(cs_h, HIGH);

        /* Put the hi-g values into the raw data array if necessary */
        for (uint8_t i=6;i<12);i=i+2){
    
            if (saturate & (1<<(i+6)){

                // add calib
                raw->a[i/2] = ((buf[sc] << 8) | buf[sc+1]) + calib[(i/2)+12];

            }

        }
        
    }

}

void imu_triple::read_float(imu_float *flt){

    imu_raw raw;
    uint16_t saturate;

    /* Get raw data and saturation bitfield */
    imu_triple.read_raw(&raw, saturate);

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

        if((1<<i)&saturate){

            if((1<<(i+3))&saturate){
                flt->a[i] = raw.a[i]*LSM6DSL_C_ACC_CONV;
            }
            else{
                flt->a[i] = raw.a[i]*LSM6DSL_C_ACC_CONV;
            }

        }
        else{
            flt->a[i] = raw.a[i]*LSM6DSL_F_ACC_CONV;
        }

    }

}

void imu_triple::read_float(imu_float *flt, imu_raw *raw, uint16_t saturate){

    /* Get raw data and saturation bitfield */
    imu_triple.read_raw(&raw, saturate);

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

        if((1<<i)&saturate){

            if((1<<(i+3))&saturate){
                flt->a[i] = raw.a[i]*LSM6DSL_C_ACC_CONV;
            }
            else{
                flt->a[i] = raw.a[i]*LSM6DSL_C_ACC_CONV;
            }

        }
        else{
            flt->a[i] = raw.a[i]*LSM6DSL_F_ACC_CONV;
        }

    }

}
