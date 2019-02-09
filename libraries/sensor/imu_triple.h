/* Cal Poly Space Systems
 * 
 * lsm6dsl.h 
 *
 */

#ifndef _LSM6DSL_PAIR_H
#define _LSM6DSL_PAIR_H

#include <Arduino.h>

/* Register definitions */
#define LSM6DSL_WHO_AM_I        0x0F
#define LSM6DSL_CTRL1_XL        0x10
#define LSM6DSL_CTRL2_G         0x11
#define LSM6DSL_OUT_START       0x22
#define LSM6DSL_OUT_TEMP        0x20

#define H3LIS331DL_WHO_AM_I     0x0F
#define H3LIS331DL_CTRL1        0x10
#define H3LIS331DL_CTRL4        0x11
#define H3LIS331DL_OUT_START    0x28

/* Linear offsets (m) */
#define LSM6DSL_C_X         -0.015
#define LSM6DSL_C_Y         0
#define LSM6DSL_C_Z         0

#define H3LIS331DL_C_X      0
#define H3LIS331DL_C_Y      -0.015
#define H3LIS331DL_C_Z      0

/* Data structures */
typedef union {

    struct s{
        int16_t gyr_xr;
        int16_t gyr_yr;
        int16_t gyr_zr;
        int16_t acc_xr;
        int16_t acc_yr;
        int16_t acc_zr;
    }__attribute__((packed));

    int16_t a[6];

} imu_raw;

typedef union{

    struct s{
        float gyr_xf;
        float gyr_yf;
        float gyr_zf;
        float acc_xf;
        float acc_yf;
        float acc_zf;
    }__attribute__((packed));

    float a[6];

} imu_float;

typedef union{

    struct s{

        int8_t gyrf_x;
        int8_t gyrf_y;
        int8_t gyrf_z;
        int8_t accf_x;
        int8_t accf_y;
        int8_t accf_z;
    
        int8_t gyrc_x;
        int8_t gyrc_y;
        int8_t gyrc_z;
        int8_t accc_x;
        int8_t accc_y;
        int8_t accc_z;

        int8_t acch_x;
        int8_t acch_y;
        int8_t acch_z;

    }

    int8_t a[15];

} imu_calib


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

typedef enum{
    LSM6DSL_FINE;
    LSM6DSL_COARSE;
    H3LIS331DL;
} imu_device_t

typedef enum{
    IMU_GYR_X;
    IMU_GYR_Y;
    IMU_GYR_Z;
    IMU_ACC_X;
    IMU_ACC_Y;
    IMU_ACC_Z;
} axis_t;

/* Sensor Class */
class imu_triple
{
	public:
		
        imu_triple(int cs_fine, int cs_coarse, int cs_hi_g);
        void set(imu_device_t dev, uint8_t addr, uint8_t value);
        uint8_t get(imu_device_t dev, uint8_t addr);
        void set_calib(imu_device_t dev, int16_t *calib);
        int16_t get_axis_raw(imu_device_t dev, axis_t axis);
        void read_raw(union imu_raw);
        void read_float(union imu_float);
        void read_float(union imu_float, union imu_raw);

	private:

        int16_t[6] calib_fine;
        int16_t[6] calib_coarse;
        int16_t[3] calib_hi_g;
        int cs_f;
        int cs_c;
        int cs_h;
        uint8_t saturate;
        float[2]    convert_fine;
        float[2]    convert_coarse;
        float       convert_hi_g;

};

#endif