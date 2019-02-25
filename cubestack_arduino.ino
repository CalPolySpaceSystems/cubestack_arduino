// Cubestack

/** 
 *  Task Scheduler for CPSS Data Collection
 *
 *  The CPSS data collection and recording system will run using the Arduino TaskScheduler library
 *  
 *  The tasks are listed in the CPSS Data Collection Task Tree.xlsx
 *  
 */

// Define operating modes
//#define DEBUG
//#define CALIB_CONTINUE

// Define task scheduler resolution
#define _TASK_MICRO_RES
 
// External Libraries
#include <TaskScheduler.h>

// Sensor Libraries
#include "sensor/imu_triple.h"
#include "sensor/lis2mdl.h"
//#include "sensor/ms56xx.h"
//#include "sensor/ubx8.h"

// Other libraries

// Pin Definitions
#define CS_IMU_FINE       2
#define CS_IMU_COARSE     3
#define CS_IMU_HI_G       4

// Sensor Class Initializations
imu_triple imu_data = imu_triple(CS_IMU_FINE,CS_IMU_COARSE,CS_IMU_HI_G);

// Sensor Data Structures
//imu_raw     i_raw;  // Unneeded for IMU/Filtering
imu_float   i_flt;
mag_float   m_flt;

// Other Variables
uint16_t sat;

// Callback methods for Task Scheduler
void imuCallback();
//void gyroCallback();
void magCallback();
//void baroCallback();
//void gpsCallback();
void sendCallback();
void saveCallback();

// Task Definitions
// Task (intervals X 100 ms) for testing
Task pollImu(1000, TASK_FOREVER, &imuCallback);
Task pollMagnatometer(10000, TASK_FOREVER, &magCallback);
//Task pollBarometer(30000, TASK_FOREVER, &baroCallback);
//Task pollGPS(50000, TASK_FOREVER, &gpsCallback);
//Task sendData(10000, TASK_FOREVER, &sendCallback);
//Task saveData(10000, TASK_FOREVER, &saveCallback);

Scheduler runner;

void imuCallback() {
    Serial.print("pollAccelerometer X: ");
    Serial.println(millis());

    Serial.print("pollAccelerometer Y: ");
    Serial.println(millis());

    Serial.print("pollAccelerometer Z: ");
    Serial.println(millis());
}


void magCallback() {
    Serial.print("pollMagnatometer X: ");
    Serial.println(millis());

    Serial.print("pollMagnatometer X: ");
    Serial.println(millis());

    Serial.print("pollMagnatometer X: ");
    Serial.println(millis()); 
}

/*
void baroCallback() {
  
    Serial.print("pollBarometer Pressure: ");
    Serial.println(millis());

    Serial.print("pollBarometer Altitude: ");
    Serial.println(millis());
}

void gpsCallback() {
    Serial.print("pollGPS Time: ");
    Serial.println(millis());

    Serial.print("pollGPS Latitude: ");
    Serial.println(millis());

    Serial.print("pollGPS Longitude: ");
    Serial.println(millis());
    
    Serial.print("pollGPS Validation: ");
    Serial.println(millis());
}
*/

void sendCallback() {
    Serial.print("sendData Compile Packet: ");
    Serial.println(millis());

    Serial.print("sendData Send Packet: ");
    Serial.println(millis());
}

void saveCallback() {
    Serial.print("saveData Save Data to SD: ");
    Serial.println(millis());
}

/* Setup */
void setup () {

  /* Perform calibration if connected to USB */




  /* Save calibration data */




  
  Serial.begin(9600);
  Serial.println("Scheduler TEST");

  // Initialize Tasks and Scheduler
  runner.init();
  Serial.println("Initialized scheduler");
  
  runner.addTask(pollAccelerometer);
  Serial.println("added pollAccelerometer");
  
  runner.addTask(pollGyroscope);
  Serial.println("added pollGyroscope");

  runner.addTask(pollMagnatometer);
  Serial.println("added pollMagnatometer");

  runner.addTask(pollBarometer);
  Serial.println("added pollBarometer");

  runner.addTask(pollGPS);
  Serial.println("added pollGPS");

  runner.addTask(sendData);
  Serial.println("added sendData");

  runner.addTask(saveData);
  Serial.println("added saveData");

  delay(5000);

  // Enable Tasks
  pollAccelerometer.enable();
  Serial.println("Enabled pollAccelerometer");
  
  pollGyroscope.enable();
  Serial.println("Enabled pollGyroscope");
  
  pollMagnatometer.enable();
  Serial.println("Enabled pollMagnatometer");
  
  pollBarometer.enable();
  Serial.println("Enabled pollBarometer");
  
  pollGPS.enable();
  Serial.println("Enabled pollGPS");
  
  sendData.enable();
  Serial.println("Enabled sendData");
  
  saveData.enable();
  Serial.println("Enabled saveData");

}

/* Loop just runs the scheduler */
void loop () {
  runner.execute();
}
