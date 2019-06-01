#ifndef _MADGWICK_MARG_H
#define _MADGWICK_MARG_H

#include <Arduino.h>

#include "../sensor/imu_triple.h"
#include "../sensor/lis2mdl.h"

/* Constants */
#define DT              0.019224f
#define ERR_GYR_DEG     3.0f
#define ERR_GYR         3.14159265358979f * (ERR_GYR_DEG / 180.0f)
#define DRIFT_GYR_DEG   0.01f
#define DRIFT_GYR       3.14159265358979f * (DRIFT_GYR_DEG / 180.0f)
#define BETA            sqrt(3.0f / 4.0f) * ERR_GYR
#define ZETA            sqrt(3.0f / 4.0f) * DRIFT_GYR

void filter_update(float q[], imu_float *imu, imu_raw *raw, mag_float *mag);

void conv_q_rpy(float q[], float ang[]);

#endif