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
//#define MODE_DEBUG
//#define MODE_COATS
//#define MODE_FLASH

// Define other parameters
#define BOARD_ZERO

// Define task scheduler resolution
#define _TASK_MICRO_RES
 
// External Libraries
#include <TaskScheduler.h>
#include <FlashStorage.h>

// Sensor Libraries
#include "sensor/imu_triple.h"
#include "sensor/lis2mdl.h"
//#include "sensor/ms56xx.h"
//#include "sensor/ubx8.h"




// Other libraries
#include "utilities/sensor_utils.h"
#include "utilities/io_utils.h"


// Pin Definitions
#define CS_IMU_FINE       2             // SPI Chip Select for the Fine LSM6DSL Accelerometer/Gyroscope
#define CS_IMU_COARSE     3             // SPI Chip Select for the Coarse LSM6DSL Accelerometer/Gyroscope
#define CS_IMU_HI_G       4             // SPI Chip Select for the H3LIS331DL High-G Accelerometer
#define CS_FLASH          8             // SPI Chip Select for the SST26VF064B Flash Memory Chip
#define HOLD_FLASH        6             // Digital Pin for the SST26VF064B Flash Memory HOLD Pin 
#define CS_SD             9             // SPI Chip Select for the SD Card Slot

#define IO_LS             12            // Digital Pin for the loudspeaker
#define IO_LED            LED_BUILTIN   // Digital Pin for the LED

/* Class definitions */
Scheduler runner;

#ifdef MODE_COATS
coats telemetry = coats(ENDING,COUNTER_DIS);
#endif

#ifdef MODE_SERIAL2
Uart Serial2 (&sercom2, 3, 2; SERCOM_RX_PAD_1, UART_TX_PAD_2);
#endif

/* Sensor Class Initializations */
imu_triple imu = imu_triple(CS_IMU_FINE,CS_IMU_COARSE,CS_IMU_HI_G);




/* Sensor Data Structures */
//imu_raw     i_raw;  // Unneeded for IMU/Filtering
imu_float   i_flt;
mag_float   m_flt;

imu_calib   i_cal;
mag_calib   m_cal;

/* Sensor Reading Times */
uint32_t imu_micros;
uint32_t mag_micros;
uint32_t baro_micros;
uint32_t 

/* Flash Library */
#ifdef ARDUINO_SAMD_ZERO
  FlashStorage(imu_calib_store,imu_calib);
  FlashStorage(mag_calib_store,mag_calib);
#endif

/* Other Variables */
// Needs to be part of imu structure?
uint16_t imu_sat;

/* Prototype callback methods for Task Scheduler */
// unneccessary if externally included?
void imuCallback();
void magCallback();
//void baroCallback();
//void gpsCallback();
//void tapCallback();

/* Polling Task Definitions */
Task pollImu(1000, TASK_FOREVER, &imuCallback);
Task pollMag(10000, TASK_FOREVER, &magCallback);
//Task pollBaro(30000, TASK_FOREVER, &baroCallback);
//Task pollGPS(50000, TASK_FOREVER, &gpsCallback);
//Task pollTap(#,#,#);

/* Telemetry Task Definitions */

/* Flash Task Definitions */

/* Other Task Definitions */
Task stateEst(1000, TASK_FOREVER, &stateEstCallback);

/*____________________Setup___________________*/
void setup () {

  /* Open all communication ports */
  SerialUSB.begin(500000);
  SPI.begin();
  Wire.begin();

  /* Initialize Sensors */
  imu.init();
  mag.init();
  //baro.init();
  

  /* Perform calibration if connected to USB */
  for (int secs=0;secs<CALIB_TIMEOUT;secs++){

    if (SerialUSB){
      
      
      
      


    #ifndef DEBUG

      SerialUSB.end();

      /* Beep to indicate completion of calibration + turnoff */

      /* Wait forever */
      while(1){};
    
    #endif
      
    }

    delay(1000);
  }

  

  // Beep to indicate end of calibration wait
  // beep();


  



  #ifdef MODE_COATS
  
  /* Initialize coats telemetry */
  telemetry.addTlm(IMU_ID,&imu_data.a,sizeof(imu_data));
  telemetry.addTlm(MAG_ID,&imu_data.a,sizeof(imu_data));
  telemetry.addTlm(BARO_ID,&imu_data.a,sizeof(imu_data));
  telemetry.addTlm(GPS_ID,&imu_data.a,sizeof(imu_data));
  telemetry.addTlm(TAP_ID,&imu_data.a,sizeof(imu_data));

  telemetry.serialInit(Serial1,230400);

  #endif
  
  /* Initialize Tasks and Scheduler */
  runner.init();

  runner.addTask(pollImu);
  runner.addTask(pollMag);
  runner.addTask(pollBaro);
  //runner.addTask(pollGPS);

  #ifdef MODE_COATS
  runner.addtask(tlmImu);
  runner.addtask(tlmMag);
  runner.addtask(tlmBaro);
  runner.addtask(tlmGPS);
  runner.addtask(tlmTap);
  runner.addtask(tlmStat);
  #endif 

  #ifdef MODE_FLASH
  runner.addTask(saveData);
  #endif

  /* Enable Tasks */
  pollImu.enable();
  pollMag.enable();
  pollBaro.enable();
  pollGPS.enable();


  #ifdef MODE_COATS
  tlmImu.enable();
  tlmMag.enable();
  tlmBaro.enable();
  tlmGPS.enable();
  tlmTap.enable();
  tlmStat.enable();
  #endif  

  #ifdef MODE_FLASH
  flashImu.enable();
  flashMag.enable();
  flashBaro.enable();
  flashGPS.enable();
  flashTap.enable();
  #endif

  stateEst.enable();
  
}

/*______________Loop just runs the scheduler_____________*/
void loop () {
  runner.execute();
}

/*______________Sensor Polling Callbacks_________________*/




/*______________Telemetry Callbacks______________________*/



/*_______________Flash Recording Callbacks_______________*/
