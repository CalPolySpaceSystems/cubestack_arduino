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
//#define MODE_DEBUG
#define MODE_COATS
//#define MODE_FLASH

/* Define task scheduler resolution */
#define _TASK_MICRO_RES


/* External Libraries */
#include <TaskScheduler.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
//#include <FlashStorage.h>

#include "SparkFun_Ublox_Arduino_Library.h"

/* Sensor Libraries */
#include "src/sensor/imu_triple.h"
#include "src/sensor/lis2mdl.h"
#include "src/sensor/ms56xx.h"

/* Other libraries */
//#include "src/util/sensor_utils.h"
#include "src/util/io_utils.h"
#include "src/state estimation/madgwick_marg.h"

/* COATS */
#include "src/external/coats/coats.h"

/* Calibration Values */
int16_t calib_imu_fine[6] = {0,0,0,0,0,0};
int16_t calib_imu_coarse[6] = {0,0,0,0,0,0};
int16_t calib_imu_hi_g[3] = {0,0,0};
int16_t calib_mag[3] = {0,0,0};

/* Pin Definitions */
#define CS_IMU_FINE       A3             // SPI Chip Select for the Fine LSM6DSL Accelerometer/Gyroscope
#define CS_IMU_COARSE     A4             // SPI Chip Select for the Coarse LSM6DSL Accelerometer/Gyroscope
#define CS_IMU_HI_G       A5             // SPI Chip Select for the H3LIS331DL High-G Accelerometer
#define CS_FLASH          5             // SPI Chip Select for the SST26VF064B Flash Memory Chip
#define HOLD_FLASH        4             // Digital Pin for the SST26VF064B Flash Memory HOLD Pin 
#define CS_SD             4             // SPI Chip Select for the SD Card Slot

#define IO_LS             13            // Digital Pin for the loudspeaker
//#define IO_LED            LED_BUILTIN   // Digital Pin for the LED

/* Other operational definitions */
#define CALIB_READS       32
#define CALIB_SECONDS     30
#define CALIB_TIMEOUT     1

/* Class definitions */
Scheduler runner;

#ifdef MODE_COATS

#define SerialDownlink  Serial

#define STAT_ID         129
#define TEMP_ID         131
#define GPS_ID          132
#define IMU_ID          133
#define IMU_RAW_ID      134
#define MAG_ID          135
#define MAG_RAW_ID      136
#define ALT_ID          137
#define BARO_ID         138
#define BARO_RAW_ID     139
#define BARO_PROM_ID    140

#define END             0x4350

coats downlink = coats(END,false,true);

#endif

Uart Serial2 (&sercom2, 3, 2, SERCOM_RX_PAD_1, UART_TX_PAD_2);

/* Sensor Class Initializations */
imu_triple      imu = imu_triple(CS_IMU_FINE,CS_IMU_COARSE,CS_IMU_HI_G);
lis2mdl         mag = lis2mdl();
SFE_UBLOX_GPS   gps;

/* Sensor Data Structures */
imu_raw     i_raw;  // Unneeded for IMU/Filtering
imu_float   i_flt;

mag_raw     m_raw;
mag_float   m_flt;

typedef union {

  struct{
    uint8_t   fix;
    float     lat;
    float     lng;
    float     alt;
  } __attribute__((packed));

  char a[13];
} gps_data;

gps_data    g_data;

baro bar; //structure for barometer, added 4/19/19

/* Calibration storage structures */

typedef struct {        
  int16_t imu_fine[6];
  int16_t imu_coarse[6];
  int16_t imu_hi_g[3] ;
  int16_t mag[3];
} calib;

uint32_t micros_last;

//FlashStorage(calib_store,calib);

/* State Variable */
float q[4] = {1.0, 0.0, 0.0, 0.0};
float rpy[3] = {0.0, 0.0, 0.0};

/* Flash Library */
#ifdef ARDUINO_SAMD_ZERO
  
  //FlashStorage(imu_calib_store,imu_calib);
  //FlashStorage(mag_calib_store,mag_calib);
#endif

/* Create a struct of calibration values of type "calib" */
//calib sensor_calib;

/* Timestamp Variables */
uint32_t imu_micros;
uint32_t mag_micros;
uint32_t baro_micros;
uint32_t gps_micros;

/* Prototype callback methods for Task Scheduler */
void imuPoll();
void magPoll();
void baroPrimeTemp();
void baroPollTemp();
void baroPollPress();
void gpsPoll();
//void tapCallback();
//void margCallback();

// Donwnlink
#ifdef MODE_COATS
void imuDownlink();
void baroDownlink();
void magDownlink();
void gpsDownlink();
#endif

/* Polling Task Definitions */
Task pollImu(1200, TASK_FOREVER, &imuPoll);
Task pollMag(10000, TASK_FOREVER, &magPoll);
Task pollBaro(10000, TASK_FOREVER, &baroPrimeTemp); //uncommented this 4/19/19
Task pollGPS(100000, TASK_FOREVER, &gpsPoll);
//Task pollTap(#,#,#);

/* Telemetry Task Definitions */
#ifdef MODE_COATS
Task tlmImu(2400, TASK_FOREVER, &imuDownlink);
Task tlmBaro(40000, TASK_FOREVER, &baroDownlink);
Task tlmMag(40000, TASK_FOREVER, &magDownlink);
Task tlmGPS(100000, TASK_FOREVER, &gpsDownlink);
#endif
/* Flash Task Definitions */

/* Other Task Definitions */
//Task margEst(1200, TASK_FOREVER, &margCallback);

/*____________________Setup___________________*/
void setup () {

  /* Open all communication ports */
  SPI.begin();
  Wire.begin();
  //Serial.begin(230400);
  //SerialUSB.begin(500000);
  //SerialUSB.setTimeout(5000);
  
  /* Initialize Digital IO */
  pinMode(CS_IMU_FINE,OUTPUT);
  digitalWrite(CS_IMU_FINE,HIGH);
  
  pinMode(CS_IMU_COARSE,OUTPUT);
  digitalWrite(CS_IMU_COARSE,HIGH);
  
  pinMode(CS_IMU_HI_G,OUTPUT);
  digitalWrite(CS_IMU_HI_G,HIGH);
  
  pinMode(IO_LS,OUTPUT);
  digitalWrite(IO_LS,LOW);

  /* Initialize Sensors */
  imu.init();
  mag.init();
  initMS56xx("MS5607"); //call the initialize function with the specified barometer model

  /* Intialize GPS */
  gps.begin();
  gps.setI2COutput(COM_TYPE_UBX);

  // Set update frequency to be the same as the polling frequency
  gps.setNavigationFrequency(10,10);

  // Set it into non-blocking mode
  gps.setAutoPVT(true,10);
  
  gps.saveConfiguration();

  //beep(IO_LS,220,400);
  //beep(IO_LS,440,400);
  //beep(IO_LS,660,400);

  /* Retrieve calibration values from flash */
  //sensor_calib = calib_store.read();

  /* Calibration *
  for (int i=0;i<CALIB_TIMEOUT;i++){
    
    if (SerialUSB){
    
      delay(1000);
      
      /* IMU Calibrate *
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

       /* Magnetometer Calibration *
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
  */

  /* Beep to indicate completion of calibration + turnoff */
 // beep(IO_LS,880,200);
  //beep(IO_LS,440,200);
  //beep(IO_LS,660,200);

  /* Retrieve calibration values from flash */
 // sensor_calib = calib_store.read();

  //if(!sensor_calib.imu_fine[1]){
    //SerialUSB.println("Error: Device not calibrated.");
  //}
  
  /* Save calibration values to sensor variables *
  //SerialUSB.println(sensor_calib.imu_fine[0]);
  imu.calib_set(LSM6DSL_FINE,sensor_calib.imu_fine);
  imu.calib_set(LSM6DSL_COARSE,sensor_calib.imu_coarse);
  imu.calib_set(H3LIS331DL,sensor_calib.imu_hi_g);
  mag.calib_set(sensor_calib.mag);
  */
  #ifdef MODE_COATS
  
  /* Initialize coats telemetry */
  downlink.addTlm(IMU_ID,&i_flt.a,sizeof(i_flt));
  downlink.addTlm(MAG_ID,&m_flt.a,sizeof(m_flt));
  downlink.addTlm(IMU_RAW_ID,&i_raw.a,sizeof(i_raw));
  downlink.addTlm(MAG_RAW_ID,&m_raw.a,sizeof(m_raw));
  downlink.addTlm(BARO_ID,&bar.a,sizeof(bar));
  downlink.addTlm(GPS_ID,&g_data.a,sizeof(g_data));
  //telemetry.addTlm(TAP_ID,&imu_data.a,sizeof(imu_data));

  downlink.serialInit(SerialDownlink,230400);

  #endif
  
  /* Initialize Tasks and Scheduler */
  runner.init();

  runner.addTask(pollImu);
  runner.addTask(pollMag);
  runner.addTask(pollBaro); //uncommented this 4/19/19
  runner.addTask(pollGPS);

  #ifdef MODE_COATS
  runner.addTask(tlmImu);
  runner.addTask(tlmMag);
  runner.addTask(tlmBaro); 
  runner.addTask(tlmGPS);
  //runner.addtask(tlmTap);
  //runner.addtask(tlmStat);
  #endif 

  #ifdef MODE_FLASH
  //runner.addTask(saveData);
  #endif


  //runner.addTask(margEst);
  
 beep(IO_LS,220,100);

  /* Enable Tasks */
  pollImu.enable();
  pollMag.enable();
  pollBaro.enable();
  pollGPS.enable();

  #ifdef MODE_COATS
  tlmImu.enable();
  tlmMag.enable();
  //tlmBaro.enable();
  tlmGPS.enable();
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

  //margEst.enable();

  //SerialUSB.println("GO");
}

/*______________Loop just runs the scheduler_____________*/
void loop () {
  runner.execute();
}

/*______________Sensor Polling Callbacks_________________*/

void imuPoll() {

  imu_micros = micros();

  imu.read_float(&i_flt,&i_raw);

  #ifdef MODE_DEBUG
  SerialUSB.print(imu_micros); 
  SerialUSB.print(',');
  SerialUSB.print(i_raw.a[0]);
  SerialUSB.print(',');
  SerialUSB.print(i_raw.a[1]);
  SerialUSB.print(',');
  SerialUSB.print(i_raw.a[2]);
  SerialUSB.print(',');
  SerialUSB.print(i_raw.a[3]);
  SerialUSB.print(',');
  SerialUSB.print(i_raw.a[4]);
  SerialUSB.print(',');
  SerialUSB.println(i_raw.a[5]);
  //SerialUSB.print(',');
  //SerialUSB.print(m_flt.a[0]);
  //SerialUSB.print(',');
  //SerialUSB.print(m_flt.a[1]);
  //SerialUSB.print(',');
  //SerialUSB.println(m_flt.a[2]);
  #endif
  
}

void magPoll() {

  mag_micros = micros();

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

//added baroCallback function 4/19/19 
void baroPrimeTemp() {

  // Prime temperature readings
  primeTempMS56xx();

  pollBaro.setCallback(&baroPollTemp);
  pollBaro.delay(10000);
}

void baroPollTemp() {

  // Read the temperature
  readTempMS56xx(&bar);

  // Prime pressure reading
  primePressureMS56xx();
  
  pollBaro.setCallback(&baroPollPress);
  pollBaro.delay(10000);
}

void baroPollPress() {

  // Read pressure
  readPressureMS56xx(&bar);

  // Calculate altitude
  calcAltitudeMS56xx(&bar);

  baro_micros = micros();

  #ifdef DEBUG
  
  String output = MS56xxToString(&bar);
  SerialUSB.println(output);

  #endif
  
  pollBaro.setCallback(&baroPrimeTemp);
}

void gpsPoll(){

    gps_micros = micros();

    g_data.fix = (uint8_t) gps.getFixType();
    g_data.lat = (gps.getLatitude()/10000000.000000);
    g_data.lng = (gps.getLongitude()/10000000.000000);
    g_data.alt = (gps.getAltitude()/1000.000000);
    
    #ifdef MODE_DEBUG 

    SerialUSB.print(gps_micros);
    SerialUSB.print(',');
    SerialUSB.print(g_data.fix);
    SerialUSB.print(',');
    SerialUSB.print(g_data.lat,6);
    SerialUSB.print(',');
    SerialUSB.println(g_data.lng,6);

    #endif

}

/*______________Telemetry Callbacks______________________*/

#ifdef MODE_COATS

void imuDownlink() {

  downlink.serialWriteTlm(SerialDownlink,IMU_RAW_ID,imu_micros);
  //Serial1.println('F');
}

void magDownlink() {

  downlink.serialWriteTlm(SerialDownlink,MAG_RAW_ID,mag_micros);
    
}

void baroDownlink() {

  downlink.serialWriteTlm(SerialDownlink,BARO_ID,baro_micros);
    
}

void gpsDownlink() {

  downlink.serialWriteTlm(SerialDownlink,GPS_ID,gps_micros);
    
}

#endif
/*_______________Flash Recording Callbacks_______________*/


/*_______________State Estimation Callbacks______________*/

void margCallback() {

  uint32_t est_micros = micros();

  // Read latest IMU data
  //imu.read_float(&i_flt,&i_raw);

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
