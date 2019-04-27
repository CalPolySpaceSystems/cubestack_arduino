/* Cal Poly Space Systems
 * 
 * lsm6dsl.h 
 *
 */

#ifndef _IMU_TRIPLE_H
#define _IMU_TRIPLE_H

#include <Arduino.h>
#include <stdint.h>

/* Register definitions */
#define LSM6DSL_WHO_AM_I        0x0F
#define LSM6DSL_CTRL1_XL        0x10
#define LSM6DSL_CTRL2_G         0x11
#define LSM6DSL_OUT_START       0xA2
#define LSM6DSL_OUT_TEMP        0x20

#define H3LIS331DL_WHO_AM_I     0x0F
#define H3LIS331DL_CTRL1        0x20
#define H3LIS331DL_CTRL4        0x23
#define H3LIS331DL_OUT_START    0xE8

/* Linear offsets (m) */
#define LSM6DSL_C_X         -0.015
#define LSM6DSL_C_Y         0
#define LSM6DSL_C_Z         0

#define H3LIS331DL_C_X      0
#define H3LIS331DL_C_Y      -0.015
#define H3LIS331DL_C_Z      0

/* Conversion factors */
#define LSM6DSL_F_GYR_CONV      0.00875
#define LSM6DSL_C_GYR_CONV      0.07

#define LSM6DSL_F_ACC_CONV      0.000062
#define LSM6DSL_C_ACC_CONV      0.000493     
#define H3LIS331DL_ACC_CONV     0.00305

/* SPI Settings */
#define SPI_SPEED       10000000
#define SPI_ENDIAN      MSBFIRST
#define SPI_MODE        SPI_MODE3

/* Data structures */
typedef union {

    struct {
        int16_t gyr_x;
        int16_t gyr_y;
        int16_t gyr_z;
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
        int16_t sat;
    }__attribute__((packed));

    int16_t a[7];

} imu_raw;

typedef union{

    struct {
        float gyr_x;
        float gyr_y;
        float gyr_z;
        float acc_x;
        float acc_y;
        float acc_z;
    }__attribute__((packed));

    float a[6];

} imu_float;

/*
typedef enum{
    LSM6DSL_ACC_2G      = 0x00;
    LSM6DSL_ACC_4G      = 0x10;
    LSM6DSL_ACC_8G      = 0x11;
    LSM6DSL_ACC_16G     = 0x01;
} lsm6dsl_acc_range_t;

typedef enum{
    LSM6DSL_GYR_250     = 0x00;
    LSM6DSL_GYR_500     = 0x01;
    LSM6DSL_GYR_1K      = 0x10;
    LSM6DSL_GYR_2K      = 0x11;
} lsm6dsl_gyr_range_t;

typedef enum{
    H3LIS331DL_ACC_100G = 0x00;
    H3LIS331DL_ACC_200G = 0x10;
    H3LIS331DL_ACC_400G = 0x11;
} h3lis331dl_acc_range_t;
*/

typedef enum{
    LSM6DSL_FINE,
    LSM6DSL_COARSE,
    H3LIS331DL,
} imu_device_t;

/*
typedef enum{
    IMU_GYR_X;
    IMU_GYR_Y;
    IMU_GYR_Z;
    IMU_ACC_X;
    IMU_ACC_Y;
    IMU_ACC_Z;
} axis_t;
*/

/* Sensor Class */
class imu_triple
{
	public:
		
        imu_triple(int cs_fine, int cs_coarse, int cs_hi_g);
        void init();
        void set(imu_device_t dev, uint8_t addr, uint8_t value);
        void calibrate(int n_samples);
        void calib_get(imu_device_t dev, int16_t calib[]);
        void calib_set(imu_device_t dev, int16_t calib[]);
        uint8_t get(imu_device_t dev, uint8_t addr);
        void read_dev(imu_device_t dev, imu_raw *raw);
        void read_raw(imu_raw *raw);
        void read_float(imu_float *flt);
        void read_float(imu_float *flt, imu_raw *raw);

	private:

        int16_t calib_fine[6];
        int16_t calib_coarse[6];
        int16_t calib_hi_g[3] ;
        int cs_f;
        int cs_c;
        int cs_h;

};

#endif
