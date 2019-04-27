// Cubestack

/** 
 *  Task Scheduler for CPSS Data Collection
 *
 *  The CPSS data collection and recording system will run using the Arduino TaskScheduler library
 *  
 *  The tasks are listed in the CPSS Data Collection Task Tree.xlsx
 *  
 */

/* Define operating modes */
#define MODE_DEBUG
//#define MODE_COATS
//#define MODE_FLASH

/* Define task scheduler resolution */
#define _TASK_MICRO_RES
 
/* External Libraries */
#include <TaskScheduler.h>
#include <SPI.h>
#include <Wire.h>
#include <FlashStorage.h>

/* Sensor Libraries */
#include "src/sensor/imu_triple.h"
#include "src/sensor/lis2mdl.h"
//#include "sensor/ms56xx.h"
//#include "sensor/ubx8.h"

/* Other libraries */
//#include "src/util/sensor_utils.h"
#include "src/util/io_utils.h"
//#include "src/state estimation/madgwick_marg.h"

/* COATS */
#include "external/coats.h"

/* Calibration Values */
int16_t calib_imu_fine[6] = {0,0,0,0,0,0};
int16_t calib_imu_coarse[6] = {0,0,0,0,0,0};
int16_t calib_imu_hi_g[3] = {0,0,0};
int16_t calib_mag[3] = {0,0,0};

/* Pin Definitions */
#define CS_IMU_FINE       2             // SPI Chip Select for the Fine LSM6DSL Accelerometer/Gyroscope
#define CS_IMU_COARSE     3             // SPI Chip Select for the Coarse LSM6DSL Accelerometer/Gyroscope
#define CS_IMU_HI_G       4             // SPI Chip Select for the H3LIS331DL High-G Accelerometer
#define CS_FLASH          8             // SPI Chip Select for the SST26VF064B Flash Memory Chip
#define HOLD_FLASH        6             // Digital Pin for the SST26VF064B Flash Memory HOLD Pin 
#define CS_SD             9             // SPI Chip Select for the SD Card Slot

#define IO_LS             7            // Digital Pin for the loudspeaker
#define IO_LED            LED_BUILTIN   // Digital Pin for the LED

/* Other operational definitions */
#define CALIB_READS       32
#define CALIB_SECONDS     30
#define CALIB_TIMEOUT     5

/* Class definitions */
Scheduler runner;

#ifdef MODE_COATS
coats telemetry = coats(ENDING,false);
#endif

#ifdef MODE_SERIAL2
Uart Serial2 (&sercom2, 3, 2; SERCOM_RX_PAD_1, UART_TX_PAD_2);
#endif

/* Sensor Class Initializations */
imu_triple  imu = imu_triple(CS_IMU_FINE,CS_IMU_COARSE,CS_IMU_HI_G);
lis2mdl     mag = lis2mdl();

/* Sensor Data Structures */
imu_raw     i_raw;  // Unneeded for IMU/Filtering
imu_float   i_flt;

mag_raw     m_raw;
mag_float   m_flt;

//imu_calib   i_cal;
//mag_calib   m_cal;

/* Calibration storage structures */

typedef struct {        
  int16_t imu_fine[6];
  int16_t imu_coarse[6];
  int16_t imu_hi_g[3] ;
  int16_t mag[3];
} calib;

uint32_t micros_last;

FlashStorage(calib_store,calib);

/* State Variable */
float q[4] = {1.0, 0.0, 0.0, 0.0};
float rpy[3] = {0.0, 0.0, 0.0};

/* Flash Library */
#ifdef ARDUINO_SAMD_ZERO
  
  //FlashStorage(imu_calib_store,imu_calib);
  //FlashStorage(mag_calib_store,mag_calib);
#endif

/* Create a struct of calibration values of type "calib" */
calib sensor_calib;

/* Other Variables */
// Needs to be part of imu structure?
uint16_t imu_sat;


/* Prototype callback methods for Task Scheduler */
// unneccessary if externally included?
//void imuCallback();
void magCallback();
//void baroCallback();
//void gpsCallback();
//void tapCallback();
void margCallback();

/* Polling Task Definitions */
Task pollImu(1200, TASK_FOREVER, &imuCallback);
Task pollMag(10000, TASK_FOREVER, &magCallback);
//Task pollBaro(30000, TASK_FOREVER, &baroCallback);
//Task pollGPS(50000, TASK_FOREVER, &gpsCallback);
//Task pollTap(#,#,#);

/* Telemetry Task Definitions */

/* Flash Task Definitions */

/* Other Task Definitions */
//Task margEst(1200, TASK_FOREVER, &margCallback);

/*____________________Setup___________________*/
void setup () {

  /* Open all communication ports */
  SPI.begin();
  Wire.begin();
  SerialUSB.begin(500000);
  SerialUSB.setTimeout(60000);
  
  /* Initialize Digital IO */
  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);
  
  pinMode(3,OUTPUT);
  digitalWrite(3,HIGH);
  
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
  
  pinMode(IO_LS,OUTPUT);
  digitalWrite(IO_LS,LOW);

  
  /* Initialize coats telemetry */
  #ifdef MODE_COATS
  
  //telemetry.addTlm(IMU_ID,&imu_data.a,sizeof(imu_data));
  //telemetry.addTlm(MAG_ID,&imu_data.a,sizeof(imu_data));
  //telemetry.addTlm(BARO_ID,&imu_data.a,sizeof(imu_data));
  //telemetry.addTlm(GPS_ID,&imu_data.a,sizeof(imu_data));
  //telemetry.addTlm(TAP_ID,&imu_data.a,sizeof(imu_data));

  //telemetry.serialInit(Serial1,230400);

  // Transmit startup status

  #endif
  
  beep(IO_LS,220,400);
  
  /* Initialize Sensors */
  imu.init();
  mag.init();

  beep(IO_LS,440,400);

  /* Initialize External Flash */

  #ifdef MODE_FLASH


  #endif

  beep(IO_LS,660,400);

  /* Retrieve calibration values from flash */
  sensor_calib = calib_store.read();

  /* Calibration */
  for (int i=0;i<CALIB_TIMEOUT;i++){
    
    if (SerialUSB){
    
      delay(500);

      /* Flash Dump */
      // Check for flash data

      // Print instructions
      SerialUSB.println("Level the unit and press any key to begin calibration."); 
      while(!SerialUSB.available()){}

      
      /* IMU Calibrate */
      // Display instructions
      SerialUSB.println("Level the unit and press any key to begin calibration."); 
      while(!SerialUSB.available()){}

      // Perform calibration
      imu.calibrate(CALIB_READS);
      
      // Beep to indicate end of IMU calibration
      beep(IO_LS,440,1000);

      // Get the values to display them
      imu.calib_get(LSM6DSL_FINE,sensor_calib.imu_fine);
      imu.calib_get(LSM6DSL_COARSE,sensor_calib.imu_coarse);
      imu.calib_get(H3LIS331DL,sensor_calib.imu_hi_g);
      
      // Update serial monitor
      SerialUSB.println("IMU:");
      
      SerialUSB.print(sensor_calib.imu_fine[0]);
      SerialUSB.print(',');
      SerialUSB.print(sensor_calib.imu_fine[1]);
      SerialUSB.print(',');
      SerialUSB.print(sensor_calib.imu_fine[2]);
      SerialUSB.print(',');
      SerialUSB.print(sensor_calib.imu_fine[3]);
      SerialUSB.print(',');
      SerialUSB.print(sensor_calib.imu_fine[4]);
      SerialUSB.print(',');
      SerialUSB.println(sensor_calib.imu_fine[5]);
      
      SerialUSB.print(sensor_calib.imu_coarse[0]);
      SerialUSB.print(',');
      SerialUSB.print(sensor_calib.imu_coarse[1]);
      SerialUSB.print(',');
      SerialUSB.print(sensor_calib.imu_coarse[2]);
      SerialUSB.print(',');
      SerialUSB.print(sensor_calib.imu_coarse[3]);
      SerialUSB.print(',');
      SerialUSB.print(sensor_calib.imu_coarse[4]);
      SerialUSB.print(',');
      SerialUSB.println(sensor_calib.imu_coarse[5]);      
      
      SerialUSB.print(sensor_calib.imu_hi_g[0]);
      SerialUSB.print(',');
      SerialUSB.print(sensor_calib.imu_hi_g[1]);
      SerialUSB.print(',');
      SerialUSB.println(sensor_calib.imu_hi_g[2]);
      delay(1000);

       /* Magnetometer Calibration */
      // Display instructions
      SerialUSB.println("Rotate the unit.");
      beep(IO_LS,220,1000);

      // Perform calibration
      mag.calibrate(CALIB_SECONDS);
      
      beep(IO_LS,660,1000);
      
      mag.calib_get(sensor_calib.mag);

      // Update serial monitor
      SerialUSB.println("Magnetometer:");
      
      SerialUSB.print(sensor_calib.mag[0]);
      SerialUSB.print(',');
      SerialUSB.print(sensor_calib.mag[1]);
      SerialUSB.print(',');
      SerialUSB.println(sensor_calib.mag[2]);

      SerialUSB.println("All calibration complete.");

      // Save calibration values
      calib_store.write(sensor_calib);

      beep(IO_LS,880,200);
      beep(IO_LS,660,200);
      beep(IO_LS,220,800);

      // Wait forever
      while(1){;}

    }

    delay(1000);
  
  }


  /* Beep to indicate completion of calibration + turnoff */
  beep(IO_LS,880,200);
  beep(IO_LS,440,200);
  beep(IO_LS,660,200);

  
  while(!SerialUSB){;}

  /* Retrieve calibration values from flash */
  sensor_calib = calib_store.read();

  if(!sensor_calib.imu_fine[1]){
    SerialUSB.println("Error: Device not calibrated.");
  }
  
  /* Save calibration values to sensor variables */
  //SerialUSB.println(sensor_calib.imu_fine[0]);
  imu.calib_set(LSM6DSL_FINE,sensor_calib.imu_fine);
  imu.calib_set(LSM6DSL_COARSE,sensor_calib.imu_coarse);
  imu.calib_set(H3LIS331DL,sensor_calib.imu_hi_g);
  mag.calib_set(sensor_calib.mag);


  /* Initialize Tasks and Scheduler */
  runner.init();

  runner.addTask(pollImu);
  runner.addTask(pollMag);
  //runner.addTask(pollBaro);
  //runner.addTask(pollGPS);

  //runner.addTask(launchDetect);

  #ifdef MODE_COATS
  //runner.addtask(tlmImu);
  //runner.addtask(tlmMag);
  //runner.addtask(tlmBaro);
  //runner.addtask(tlmGPS);
  //runner.addtask(tlmTap);
  //runner.addtask(tlmStat);
  #endif 

  #ifdef MODE_FLASH
  //runner.addTask(saveData);
  #endif

  runner.addTask(margEst);

      

  beep(IO_LS,220,1000);

  /* Enable Tasks */
  //pollImu.enable();
  pollMag.enable();
  //pollBaro.enable();
  //pollGPS.enable();


  #ifdef MODE_COATS
  //tlmImu.enable();
  //tlmMag.enable();
  //tlmBaro.enable();
  //tlmGPS.enable();
  //tlmTap.enable();
  //tlmStat.enable();
  #endif  

  #ifdef MODE_FLASH
  //flashImu.enable();
  //flashMag.enable();
  //flashBaro.enable();
  //flashGPS.enable();
  //flashTap.enable();
  #endif

  margEst.enable();

  //SerialUSB.println("GO");
}

/*______________Loop just runs the scheduler_____________*/
void loop () {
  runner.execute();
}

/*______________Sensor Polling Callbacks_________________*/

void imuCallback() {

  uint32_t imu_micros = micros();

  imu_sat = imu.read_float(&i_flt,&i_raw);

  #ifdef MODE_DEBUG
  SerialUSB.print(imu_micros);
  SerialUSB.print(',');
  SerialUSB.print(i_flt.a[0]);
  SerialUSB.print(',');
  SerialUSB.print(i_flt.a[1]);
  SerialUSB.print(',');
  SerialUSB.print(i_flt.a[2]);
  SerialUSB.print(',');
  SerialUSB.print(i_flt.a[3]);
  SerialUSB.print(',');
  SerialUSB.print(i_flt.a[4]);
  SerialUSB.print(',');
  SerialUSB.print(i_flt.a[5]);
  SerialUSB.print(',');
  SerialUSB.print(m_flt.a[0]);
  SerialUSB.print(',');
  SerialUSB.print(m_flt.a[1]);
  SerialUSB.print(',');
  SerialUSB.println(m_flt.a[2]);
  #endif
  
}

void magCallback() {

 uint32_t mag_micros = micros();

  mag.read_float(&m_flt,&m_raw);

  #ifdef MODE_DEBUG
  
  // Print all data
  SerialUSB.print(mag_micros);
  SerialUSB.print(',');
  SerialUSB.print(m_flt.x);
  SerialUSB.print(',');
  SerialUSB.print(m_flt.y);
  SerialUSB.print(',');
  SerialUSB.println(m_flt.z);
  #endif
  
}

/*______________Telemetry Callbacks______________________*/



/*_______________Flash Recording Callbacks_______________*/


/*_______________State Estimation Callbacks______________*/

void launchCallback(){

  // See if continuous g-force is present

  

  // Initiate flash recording processes


  // Change rate of telemetry downlink


  // Terminate this process
  
}

void margCallback() {

  uint32_t est_micros = micros();

  // Read latest IMU data
  imu_sat = imu.read_float(&i_flt,&i_raw);

  // Update filter and calculate attitude
  filter_update(q,&i_flt,&m_flt);

  // Convert quaternians to roll/pitch/yaw
  conv_q_rpy(q,rpy);

  #ifdef MODE_DEBUG
  
  // Print all data
  SerialUSB.print(est_micros);
  SerialUSB.print(',');
  SerialUSB.print(rpy[0]);
  SerialUSB.print(',');
  SerialUSB.print(rpy[1]);
  SerialUSB.print(',');
  SerialUSB.println(rpy[2]);
  #endif
  
}
