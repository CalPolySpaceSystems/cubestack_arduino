
#include <Arduino.h>
#include "madgwick_marg.h"

float b_x = 1;
float b_z = 0; // reference direction of flux in earth frame
float w_bx = 0;
float w_by = 0;
float w_bz = 0; // estimate gyroscope biases error

void filter_update(float q[],imu_float *imu, imu_raw *raw, mag_float *mag){

    // convert rotations to rad/s
    float w_x = imu->gyr_x*0.01745;
    float w_y = imu->gyr_y*0.01745;
    float w_z = imu->gyr_z*0.01745;

    // Pull out saturate value
    uint16_t sat = raw->sat;

    float norm;
    float qdotw[4];
    float f[6];
    float J1124, J1223, J1322, J1421, J32, J33, J41, J42, J43, J44,
    J51, J52, J53, J54, J61, J62, J63, J64;
    float q_err[4];
    float w_err[3];
    float b[3];

    // Auxiliary variables to avoid repeated calculations 
    float q0_5 = 0.5f * q[0]; 
    float q1_5 = 0.5f * q[1];
    float q2_5 = 0.5f * q[2];
    float q3_5 = 0.5f * q[3];

    float q0_2 = 2.0f * q[0]; 
    float q1_2 = 2.0f * q[1];
    float q2_2 = 2.0f * q[2];
    float q3_2 = 2.0f * q[3];

    float b2_x = 2.0f * b[0];
    float b2_z = 2.0f * b[2];

    float b2x_q0 = b2_x * q[0];
    float b2x_q1 = b2_x * q[1];
    float b2x_q2 = b2_x * q[2];
    float b2x_q3 = b2_x * q[3];

    float b2z_q0 = b2_z * q[0];
    float b2z_q1 = b2_z * q[1];
    float b2z_q2 = b2_z * q[2];
    float b2z_q3 = b2_z * q[3];

    float q0_q1;
    float q0_q2 = q[0]*q[2];
    float q0_q3;
    float q1_q2;
    float q1_q3 = q[1]*q[3];
    float q2_q3;

    float adj_y = 0.0000;
    float adj_x = 0.0000;

    // Account for acceleration
    if (sat & (1<<4)){
        adj_y = 0.001529052*((w_x*w_x)*(w_z*w_z));
    }

    if (sat & (1<<6)){
        adj_x = 0.001529052*((w_x*w_x)+(w_z*w_z));
    }
    
    float a_x = (imu->acc_x)-adj_x; 
    float a_y = (imu->acc_y)+adj_y; 
    float a_z = imu->acc_z; 

    // Normalize the accelerometer measurement 
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z); 

    // Normalize the magnetometer measurement 
    norm = sqrt(mag->x * mag->x + mag->y * mag->y + mag->z * mag->z); 
    float m_x = mag->x /norm; 
    float m_y = mag->y /norm; 
    float m_z = mag->z /norm; 
    //SerialUSB.println(norm);

    // Compute objective function and jacobian
    f[0] = q1_2 * q[3] - q0_2 * q[2] - a_x;
    f[1] = q0_2 * q[1] + q2_2 * q[3] - a_y;
    f[2]= 1.0f - q1_2 * q[1] - q2_2 * q[2] - a_z;
    f[3] = b2_x * (0.5f - q[2] * q[2] - q[3] * q[3]) + b2_z * (q[1]*q[3] - q[0]*q[2]) - m_x; 
    f[4] = b2_x * (q[1] * q[2] - q[0] * q[3]) + b2_z * (q[0] * q[1] + q[2] * q[3]) - m_y;
    f[5] = b2_x * (q[0]* q[2] + q[1]*q[3]) + b2_z * (0.5f - q[1] * q[1] - q[2] * q[2]) - m_z;

    J1124 = q2_2; // J_11 negated in matrix multiplication
    J1223 = 2.0f * q[3];
    J1322 = q0_2; // J_12 negated in matrix multiplication
    J1421 = q1_2;
    J32 = 2.0f * J1421; // negated in matrix multiplication
    J33 = 2.0f * J1124; // negated in matrix multiplication
    J41 = b2z_q2; // negated in matrix multiplication
    J42 = b2z_q3;
    J43 = 2.0f * b2x_q2 + b2z_q0; // negated in matrix multiplication
    J44 = 2.0f * b2x_q3 - b2z_q1; // negated in matrix multiplication
    J51 = b2x_q3 - b2z_q1; // negated in matrix multiplication
    J52 = b2x_q2 + b2z_q0;
    J53 = b2x_q1 + b2z_q3;
    J54 = b2x_q0 - b2z_q2; // negated in matrix multiplication
    J61 = b2x_q2; 
    J62 = b2x_q3 - 2.0f * b2z_q1;
    J63 = b2x_q0 - 2.0f * b2z_q2;
    J64 = b2x_q1;

    // Compute the gradient
    q_err[0] = J1421 * f[1] - J1124 * f[0] - J41 * f[3] - J51 * f[4] + J61 * f[5];
    q_err[1] = J1223 * f[0] + J1322 * f[1] - J32 * f[2] + J42 * f[3] + J52 * f[4] + J62 * f[5];
    q_err[2] = J1223 * f[1] - J33 * f[2] - J1322 * f[0] - J43 * f[3] + J53 * f[4] + J63 * f[5];
    q_err[3] = J1421 * f[0] + J1124 * f[1] - J44 * f[3] - J54 * f[4] + J64 * f[5];


    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(q_err[0] * q_err[0] + q_err[1] * q_err[1] + q_err[2] * q_err[2] + q_err[3] * q_err[3]);
    q_err[0] = q_err[0] / norm;
    q_err[1] = q_err[1] / norm;
    q_err[2] = q_err[2] / norm;
    q_err[3] = q_err[3] / norm;

    //SerialUSB.println(norm);

    // compute angular estimated direction of the gyroscope error
    w_err[0] = q0_2 * q_err[1] - q1_2 * q_err[0] - q2_2 * q_err[3] + q3_2 * q_err[2];
    w_err[1] = q0_2 * q_err[2] + q1_2 * q_err[3] - q2_2 * q_err[0] - q3_2 * q_err[1];
    w_err[2] = q0_2 * q_err[3] - q1_2 * q_err[2] + q2_2 * q_err[1] - q3_2 * q_err[0];

    // compute and remove the gyroscope baises
    w_bx += w_err[0] * DT * ZETA;
    w_by += w_err[1] * DT * ZETA;
    w_bz += w_err[2] * DT * ZETA;
    w_x -= w_bx;
    w_y -= w_by;
    w_z -= w_bz;

    // compute the quaternion rate measured by gyroscopes
    qdotw[0] = -q1_5 * w_x - q2_5 * w_y - q3_5 * w_z;
    qdotw[1] = q0_5 * w_x + q2_5 * w_z - q3_5 * w_y;
    qdotw[2]= q0_5 * w_y - q1_5 * w_z + q3_5 * w_x;
    qdotw[3] = q0_5 * w_z + q1_5 * w_y - q2_5 * w_x;

    // compute then integrate the estimated quaternion rate
    q[0] += ( qdotw[0] - (BETA * q_err[0])) * DT;
    q[1] += ( qdotw[1] - (BETA * q_err[1])) * DT;
    q[2] += ( qdotw[2] - (BETA * q_err[2])) * DT;
    q[3] += ( qdotw[3] - (BETA * q_err[3])) * DT;

    // normalise quaternion
    norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;

    // compute flux in the earth frame
    q0_q1 = q[0] * q[1];  // recompute axulirary variables
    q0_q2 = q[0] * q[2];
    q0_q3 = q[0] * q[3];
    q2_q3 = q[2] * q[3];
    q1_q2 = q[1] * q[2];
    q1_q3 = q[1] * q[3];
    b[0] = 2 * mag->x *(0.5f - q[2] * q[2] - q[3] * q[3]) + 2 * mag->y * (q1_q2 - q0_q3) +  2 * mag->z * (q1_q3 + q0_q2);
    b[1] = 2 * mag->x * (q1_q2 + q0_q3) + 2 * mag->y  * (0.5f - q[1] * q[1] - q[3] * q[3]) +  2 * mag->z * (q2_q3 - q0_q1);
    b[2] = 2 * mag->x * (q1_q3 - q0_q2) + 2 * mag->y * (q2_q3 + q0_q1) + 2 * mag->z * (0.5f - q[1] * q[1] - q[2] * q[2]);

    // normalise the flux vector to have only components in the x and z
    b_x = sqrt((b[0] * b[0]) + (b[1] * b[1]));
    b_z = b[2];

}

void conv_q_rpy(float q[], float ang[]){

    ang[0] = 57.2958 * atan2f((q[0]*q[1] + q[2]*q[3]) ,(0.5f - (q[1]*q[1] + q[2]*q[2]) ) );
    ang[1] = 57.2958 * asinf(2.0f * (q[0]*q[2] - q[3]*q[1]));
    ang[2] = 57.2958 * atan2f((q[1]*q[2] + q[0]*q[3]),(0.5f - q[2]*q[2] - q[3]*q[3]));

}
