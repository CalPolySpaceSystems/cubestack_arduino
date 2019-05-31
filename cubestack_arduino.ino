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
#define MODE_FLASH
#define MODE_STATE

/* Define task scheduler resolution */
#define _TASK_MICRO_RES

/* External Libraries */
#include <TaskScheduler.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <FlashStorage.h>
#include <SPIMemory.h>

#include "SparkFun_Ublox_Arduino_Library.h"

/* Sensor Libraries */
#include "src/sensor/imu_triple.h"
#include "src/sensor/lis2mdl.h"
#include "src/sensor/ms56xx.h"
//#include "src/sensor/....h"

/* Other libraries */
#include "src/util/io_utils.h"
#include "src/util/dvrvtx.h"
#include "src/state estimation/madgwick_marg.h"

/* COATS */
#include "src/external/coats/coats.h"

/* Pin Definitions */
#define CS_IMU_FINE       A3             // SPI Chip Select for the Fine LSM6DSL Accelerometer/Gyroscope
#define CS_IMU_COARSE     A4             // SPI Chip Select for the Coarse LSM6DSL Accelerometer/Gyroscope
#define CS_IMU_HI_G       A5             // SPI Chip Select for the H3LIS331DL High-G Accelerometer
#define CS_FLASH          5             // SPI Chip Select for the SST26VF064B Flash Memory Chip
#define HOLD_FLASH        4             // Digital Pin for the SST26VF064B Flash Memory HOLD Pin 
#define CS_SD             4             // SPI Chip Select for the SD Card Slot
#define IO_REC            6

#define IO_LS             13            // Digital Pin for the loudspeaker
//#define IO_LED            LED_BUILTIN   // Digital Pin for the LED

/* Other operational definitions */
#define CALIB_READS       32
#define CALIB_SECONDS     30
#define CALIB_TIMEOUT     5

/* Add additional serial port to the central bus */
Uart Serial2 (&sercom2, 3, 2, SERCOM_RX_PAD_1, UART_TX_PAD_2);

/* Class definitions */
Scheduler runner;

#if defined(MODE_COATS) || defined(MODE_FLASH) 

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
#define STATE_ID        143

#define END             0x4350

#define MODE_ID         0xA5
#define FLASH_ID        0xB4
#define VID_ID          0xC3

#define STAT_GROUND     0x10
#define STAT_LAUNCH     0x11

#define STAT_ERASED     0x41

coats downlink = coats(END,STATUS_ENABLE,CRC8);

#endif

#ifdef MODE_FLASH

SPIFlash flash(CS_FLASH);
uint32_t nextAddress = 0;
bool flashWrite = false;

#endif

#define IMU_FAST        2403
#define IMU_SLOW        4806
#define MAG_FAST        10000
#define MAG_SLOW        40000
#define BARO_GPS_FAST   50000
#define BARO_GPS_SLOW   100000
#define TAP_FAST        5000
#define TAP_SLOW        10000
#define STATE_INTERVAL  19224

/* Sensor Class Initializations */
imu_triple      imu = imu_triple(CS_IMU_FINE,CS_IMU_COARSE,CS_IMU_HI_G);
lis2mdl         mag = lis2mdl();
SFE_UBLOX_GPS   gps;

dvrvtx          vtx = dvrvtx(Serial2,IO_REC);

/* Sensor Data Structures */
imu_raw     i_raw;
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

baro        b_data; 

uint16_t p_data[6];


/* Calibration storage structures */
typedef struct {        
  int16_t imu_fine[6];
  int16_t imu_coarse[6];
  int16_t imu_hi_g[3] ;
  int16_t mag[3];
} calib;

/* State Variable */
#ifdef MODE_STATE
static float q[4] = {1.00, 0.00, 0.00, 0.00};
#endif

//float rpy[3] = {0.0, 0.0, 0.0};

/* Internal calibration storage */
#ifdef ARDUINO_SAMD_ZERO
  FlashStorage(calib_store,calib);
#endif

/* Create a struct of calibration values of type "calib" */
calib sensor_calib;

/* Timestamp Variables */
uint32_t imu_micros;
uint32_t mag_micros;
uint32_t baro_micros;
//uint32_t tap_micros
uint32_t gps_micros;
uint32_t state_micros;

/* Prototype callback methods for Task Scheduler */
void imuPoll();
void magPoll();
void baroPrimeTemp();
void baroPollTemp();
void baroPollPress();
void gpsPoll();
//void tapCallback();

#ifdef MODE_STATE
void margCallback();
#endif

/* Telemetry Callbacks */
#ifdef MODE_COATS
void imuDownlink();
void baroDownlink();
void magDownlink();
void gpsDownlink();
//void tapDownlink();
//void stateDownlink();
void cmdUplink();
void deassertRec();

/* Command Callbacks */
void setMode(uint8_t param);
void eraseFlash(uint8_t param);

#endif

void assertRec();
void deassertRec();

/* Polling Task Definitions */
Task pollImu(IMU_SLOW, TASK_FOREVER, &imuPoll);
Task pollMag(MAG_SLOW, TASK_FOREVER, &magPoll);
Task pollBaro(BARO_GPS_SLOW, TASK_FOREVER, &baroPrimeTemp);
//Task pollTap(#,#,#);
Task pollGPS(BARO_GPS_SLOW, TASK_FOREVER, &gpsPoll);

/* Telemetry Task Definitions */
#ifdef MODE_COATS
Task tlmImu(IMU_SLOW, TASK_FOREVER, &imuDownlink);
Task tlmBaro(BARO_GPS_SLOW, TASK_FOREVER, &baroDownlink);
Task tlmMag(MAG_SLOW, TASK_FOREVER, &magDownlink);
Task tlmGPS(BARO_GPS_SLOW, TASK_FOREVER, &gpsDownlink);
Task cmdRx(100000, TASK_FOREVER, &cmdUplink);
#endif

/* Other Task Definitions */
Task deassertRecBtn(100000,1,&deassertRec);
#ifdef MODE_STATE
Task margEst(STATE_INTERVAL, TASK_FOREVER, &margCallback);
#endif
/*____________________Setup___________________*/
void setup () {

  /* Status code for startup */
  #ifdef MODE_COATS
  uint8_t statusStartup = 0x20;
  #endif
  
  /* Initialize Digital IO */
  pinMode(CS_IMU_FINE,OUTPUT);
  digitalWrite(CS_IMU_FINE,HIGH);
  
  pinMode(CS_IMU_COARSE,OUTPUT);
  digitalWrite(CS_IMU_COARSE,HIGH);
  
  pinMode(CS_IMU_HI_G,OUTPUT);
  digitalWrite(CS_IMU_HI_G,HIGH);
  
  pinMode(IO_LS,OUTPUT);
  digitalWrite(IO_LS,HIGH);

  pinMode(IO_REC,OUTPUT);
  digitalWrite(IO_REC,LOW);

  beep(IO_LS,440,400);
  beep(IO_LS,659,400);
  beep(IO_LS,880,400);
  
  /* Open all communication ports */
  SPI.begin();
  Wire.begin();
  SerialUSB.begin(230400);
  
  /* Initialize Sensors */
  imu.init();
  mag.init();
  initMS56xx("MS5607"); //call the initialize function with the specified barometer model

  /* Intialize GPS */
  gps.begin();
  gps.setI2COutput(COM_TYPE_UBX);
  gps.setNavigationFrequency(10);
  gps.setAutoPVT(true);
  gps.saveConfiguration();

  #ifdef MODE_FLASH

  flash.begin();

  bool unlockFlash = 1;

  // Check for data in flash
  for (uint8_t i=0;i<16;i++){
    char b = flash.readByte(i);
    if ((uint8_t)b != 0xFF){

      beep(IO_LS,523,400);
      beep(IO_LS,554,200);
      beep(IO_LS,494,400);
      
      // Do not allow flash to be unlocked
      unlockFlash = 0;

      statusStartup |= 0x02;
      
      break;
      
    }
  }
  
  if (unlockFlash){
    
  digitalWrite(CS_FLASH,LOW);
  SPI.transfer(0x06);
  digitalWrite(CS_FLASH,HIGH);

  delay(100);

  digitalWrite(CS_FLASH,LOW);
  SPI.transfer(0x98);
  digitalWrite(CS_FLASH,HIGH);
  
  }
    
  #endif

  /* Retrieve calibration values from flash */
  sensor_calib = calib_store.read();

  /* Calibration and Flash Offload */
  for (int i=0;i<CALIB_TIMEOUT;i++){
    
    if (SerialUSB){

      #ifdef MODE_FLASH
      
      flashOffload();

      #endif
      bool isCalibrated = false;
      if(sensor_calib.imu_fine[1]){
        isCalibrated = true;
      } 
      
      calibrate(isCalibrated);
    }

    delay(1000);
  
  }

  beep(IO_LS,523,600);
  beep(IO_LS,1046,600);

  delay(300);
  
  /* Notify if uncalibrated */ 
  if(!sensor_calib.imu_fine[1]){
    
    statusStartup |= 0x01;
    beep(IO_LS,494,600);
    
  }
  
  /* Save calibration values to sensor variables */
  imu.calib_set(LSM6DSL_FINE,sensor_calib.imu_fine);
  imu.calib_set(LSM6DSL_COARSE,sensor_calib.imu_coarse);
  imu.calib_set(H3LIS331DL,sensor_calib.imu_hi_g);
  mag.calib_set(sensor_calib.mag);

  /* Turn off SerialUSB if no longer needed*/
  #ifndef MODE_DEBUG
    SerialUSB.end();
  #endif
  
  #if defined(MODE_COATS) || defined(MODE_FLASH) 
  
  /* Initialize coats telemetry */
  downlink.addTlm(IMU_ID,&i_flt.a,sizeof(i_flt));
  downlink.addTlm(MAG_ID,&m_flt.a,sizeof(m_flt));
  downlink.addTlm(IMU_RAW_ID,&i_raw.a,sizeof(i_raw));
  downlink.addTlm(MAG_RAW_ID,&m_raw.a,sizeof(m_raw));
  downlink.addTlm(BARO_ID,&b_data.a,sizeof(b_data));
  downlink.addTlm(GPS_ID,&g_data.a,sizeof(g_data));
  //downlink.addTlm(TAP_ID,&imu_data.a,sizeof(imu_data));

  #endif

  #ifdef MODE_STATE
  downlink.addTlm(STATE_ID,&q,sizeof(q));
  #endif

  #ifdef MODE_COATS

  /* Initialize COATS commands */
  downlink.addCmd(MODE_ID,&setMode);
  downlink.addCmd(FLASH_ID,&eraseFlash);
  
  /* Connect to the radio */
  downlink.serialInit(SerialDownlink,230400);
  
  // Print startup status
  downlink.serialWriteStat(SerialDownlink,statusStartup); 
  
  #endif

  /* Initialize Tasks and Scheduler */
  runner.init();

  runner.addTask(pollImu);
  runner.addTask(pollMag);
  runner.addTask(pollBaro);
  //runner.addTask(pollTap);
  runner.addTask(pollGPS);

  #ifdef MODE_STATE
  runner.addTask(margEst);
  #endif
  
  #ifdef MODE_COATS
  runner.addTask(tlmImu);
  runner.addTask(tlmMag);
  runner.addTask(tlmBaro); 
  runner.addTask(tlmGPS);
  //runner.addtask(tlmTap);
  runner.addTask(cmdRx);
  #endif 

  /* Enable Tasks */
  pollImu.enable();
  pollMag.enable();
  pollBaro.enable();
  //pollTap.enable();
  pollGPS.enable();

  #ifdef MODE_STATE
  margEst.enable();
  #endif

  #ifdef MODE_COATS
  tlmImu.enable();
  tlmMag.enable();
  tlmBaro.enable();
  tlmGPS.enable();
  //tlmTap.enable();
  cmdRx.enable();


  
  #endif  

}

/*______________Loop just runs the scheduler_____________*/
void loop () {
  runner.execute();
}

/*______________Sensor Polling Callbacks_________________*/

/* IMU */
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
  SerialUSB.print(i_raw.a[5]);
  SerialUSB.print(',');
  SerialUSB.print(m_flt.a[0]);
  SerialUSB.print(',');
  SerialUSB.print(m_flt.a[1]);
  SerialUSB.print(',');
  SerialUSB.println(m_flt.a[2]);
  #endif

  #ifdef MODE_FLASH

  if (flashWrite){
    uint8_t buf[32];
    uint8_t sz;

    sz = downlink.buildTlm(IMU_RAW_ID,buf,imu_micros);
    flash.writeByteArray(nextAddress,buf,sz,false);
    nextAddress+=(sz);
  }
  #endif
  
}

/* Magnetometer */
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

  #ifdef MODE_FLASH
  
  if (flashWrite){
    uint8_t buf[32];
    uint8_t sz;
  
    sz = downlink.buildTlm(MAG_RAW_ID,buf,imu_micros);
    flash.writeByteArray(nextAddress,buf,sz,false);
    nextAddress+=(sz);
 }
 
  #endif
  
}

/* Barometer Poll */
void baroPrimeTemp() {

  // Prime temperature readings
  primeTempMS56xx();

  pollBaro.setCallback(&baroPollTemp);
  pollBaro.delay(10000);
}

void baroPollTemp() {

  // Read the temperature
  readTempMS56xx(&b_data);

  // Prime pressure reading
  primePressureMS56xx();
  
  pollBaro.setCallback(&baroPollPress);
  pollBaro.delay(10000);
}

void baroPollPress() {

  // Read pressure
  readPressureMS56xx(&b_data);

  // Calculate altitude
  calcAltitudeMS56xx(&b_data);

  baro_micros = micros();

  #ifdef DEBUG
  String output = MS56xxToString(&b_data);
  SerialUSB.println(output);
  #endif
  
  #ifdef MODE_FLASH
  if (flashWrite){
    uint8_t buf[32];
    uint8_t sz;

    sz = downlink.buildTlm(BARO_ID,buf,imu_micros);
    flash.writeByteArray(nextAddress,buf,sz,false);
    nextAddress+=(sz);
  }   
  #endif
  
  
  pollBaro.setCallback(&baroPrimeTemp);
}

/* GPS */
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

  #ifdef MODE_FLASH
  if (flashWrite){
    uint8_t buf[32];
    uint8_t sz;

    sz = downlink.buildTlm(GPS_ID,buf,imu_micros);
    flash.writeByteArray(nextAddress,buf,sz,false);
    nextAddress+=(sz);
  }
  
  #endif
  
}
/*
void ptapCallback1() {
  raw_data_packet[0] = readRaw_ads1115(ADC_A_ADDR);
  prime_ads1115(ADC_A_ADDR, 1); //A at ch1

  raw_data_packet[1] = readRaw_ads1115(ADC_B_ADDR);
  prime_ads1115(ADC_B_ADDR, 1); //B at ch1

  pollPtap.setCallback(&ptapCallback3);
  pollPtap.delay(1000);
}

void ptapCallback2() {
  raw_data_packet[2] = readRaw_ads1115(ADC_A_ADDR);
  prime_ads1115(ADC_A_ADDR, 2); //A at ch2

  raw_data_packet[3] = readRaw_ads1115(ADC_B_ADDR);
  prime_ads1115(ADC_B_ADDR, 2); //B at ch2

  
  for(int i=0; i<=5; i++) {
    SerialUSB.println(raw_data_packet[i]);
  }

  pollPtap.setCallback(&ptapCallback1);

}

void ptapCallback3() {
  ptap_micros = micros();

  raw_data_packet[4] = readRaw_ads1115(ADC_A_ADDR);
  prime_ads1115(ADC_A_ADDR, 0); //A at ch0

  raw_data_packet[5] = readRaw_ads1115(ADC_B_ADDR);
  prime_ads1115(ADC_B_ADDR, 0); //B at ch0

  pollPtap.setCallback(&ptapCallback2);
  pollPtap.delay(1000); //calculated to be 983 microseconds, round to 1000
}
*/

/*______________Telemetry Callbacks______________________*/

#ifdef MODE_COATS

void imuDownlink() {

  downlink.serialWriteTlm(SerialDownlink,IMU_RAW_ID,imu_micros);

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

/*
void tapDownlink(){

}
*/
#ifdef MODE_STATE
void stateDownlink(){
  
  downlink.serialWriteTlm(SerialDownlink,STATE_ID,state_micros);
  
}
#endif

void cmdUplink(){

  downlink.serialParseCmd(SerialDownlink);
  
}

#endif

/*_______________State Estimation Callbacks______________*/

#ifdef MODE_STATE
void margCallback() {

  state_micros = micros();

  // Update filter and calculate attitude
  filter_update(q,&i_flt,&m_flt);

  // Convert quaternians to roll/pitch/yaw
  //conv_q_rpy(q,rpy);

  #ifdef MODE_DEBUG
  
  // Print all data
  //SerialUSB.print(est_micros);
  //SerialUSB.print(',');
  SerialUSB.print(rpy[0]);
  SerialUSB.print(',');
  SerialUSB.print(rpy[1]);
  SerialUSB.print(',');
  SerialUSB.println(rpy[2]);
  #endif

  #ifdef MODE_FASH
  if (flashWrite){
    uint8_t buf[32];
    uint8_t sz;

    sz = downlink.buildTlm(STATE_ID,&buf,imu_micros);
    flash.writeByteArray(nextAddress,&buf,sz,false);
    nextAddress+=(sz);
  }
  #endif
  
}
#endif

/*____________________Command Callbacks__________________*/

void setMode(uint8_t param){

  downlink.serialWriteStat(SerialDownlink, STAT_CMD_RX);

  switch (param){

    // Ground
    case 0x10:

      pollImu.setInterval(IMU_SLOW);
      pollMag.setInterval(MAG_SLOW);
      pollBaro.setInterval(BARO_GPS_SLOW);
      pollGPS.setInterval(BARO_GPS_SLOW);
      //pollTap.setInterval(PTAP*2);

      #ifdef MODE_COATS
      tlmImu.setInterval(IMU_SLOW);
      tlmBaro.setInterval(BARO_GPS_SLOW);
      tlmMag.setInterval(MAG_SLOW);
      tlmGPS.setInterval(BARO_GPS_SLOW);
      //tlmTap.setInterval(PTAP*4);
      
      // Send status
      downlink.serialWriteStat(SerialDownlink,STAT_GROUND);
      #endif

      #ifdef MODE_FLASH

      flashWrite = false;

      #endif

      // Set video power
      // vtx.setPower(PWR_25,1);

      // Toggle IO
      digitalWrite(IO_REC,HIGH);
  
      // Make a task to toggle IO again in 250 ms
      runner.addTask(deassertRecBtn);
      deassertRecBtn.enable();
      
      break;

    // Launch
    case 0x11:
      
      pollImu.setInterval(IMU_FAST);
      pollMag.setInterval(MAG_FAST);
      pollBaro.setInterval(BARO_GPS_FAST);
      pollGPS.setInterval(BARO_GPS_FAST);
      //pollTap.setInterval(PTAP)

      #ifdef MODE_COATS
      tlmImu.setInterval(IMU_FAST);
      tlmBaro.setInterval(BARO_GPS_FAST);
      tlmMag.setInterval(MAG_FAST*2);
      tlmGPS.setInterval(BARO_GPS_FAST);
      //tlmTap.setInterval(PTAP*2);

      // Send status
      downlink.serialWriteStat(SerialDownlink,STAT_LAUNCH);
      #endif

      #ifdef MODE_FLASH

      flashWrite = true;
  
      #endif

      // Set video power
      //vtx.setPower(PWR_500,1);

      // Toggle IO
      digitalWrite(IO_REC,HIGH);
  
      // Make a task to toggle IO again in 250 ms
      runner.addTask(deassertRecBtn);
      deassertRecBtn.enable();

      break;

    default:

      #ifdef MODE_COATS
        downlink.serialWriteStat(SerialDownlink,STAT_INVALID);  
      #endif

  }
   
}

void eraseFlash(uint8_t param){

  if (param == 0x21){

  #ifdef MODE_FLASH
    
    digitalWrite(CS_FLASH,LOW);
    SPI.transfer(0x06);
    digitalWrite(CS_FLASH,HIGH);

    digitalWrite(CS_FLASH,LOW);
    SPI.transfer(0x98);
    digitalWrite(CS_FLASH,HIGH);
    
    flash.eraseChip();

  #endif

  downlink.serialWriteStat(SerialDownlink,STAT_ERASED);
    
  }
  
}

/*____________________Other Functions__________________*/

void deassertRec(){

  // Toggle IO
  digitalWrite(IO_REC,LOW);

  // Delete task
  runner.deleteTask(deassertRecBtn);
  
}

#ifdef MODE_FLASH

void flashOffload(){

  SerialUSB.println("Data detected in storage. Would you like to offload it? (y/n)"); 
  
  while(1){
    while(!SerialUSB.available()){} 

    char in = SerialUSB.read();

    //SerialUSB.println(in);

    if (in == 'y'){

      SerialUSB.println("Offloading Flash...");

      SerialDownlink.begin(230400);

      uint8_t emptyCount = 0;

      for (uint32_t i=0;i<8000000;i++){

        char c = flash.readChar(i);
        SerialDownlink.write(c);
        if (c==0xFF){
          emptyCount++;
        }
        else{
          emptyCount = 0;
        }

        if (emptyCount>64){
          break;
        }
        
        delayMicroseconds(100);
        
      }

      SerialUSB.println("Offload Complete.");

      playInternationale();
      
      while(1){;}
      
    }

    else if (in == 'n'){
      
      break;
    }

    else {
      SerialUSB.println("Invalid input."); 
    }

    while(SerialUSB.available()){
      char c = SerialUSB.read();
    }
  
  }
  
}

#endif

void calibrate(bool hasCalibration){
  
  if (!hasCalibration){
    SerialUSB.println("Unit is not calibrated. Press any key to begin calibration."); 
  }
  else{
    SerialUSB.println("Unit has previously been calibrated. Press any key to calibrate again."); 
  }
  
  while(!SerialUSB.available()){} 

  /* IMU Calibrate */
  imu.calibrate(CALIB_READS);
  imu.calib_get(LSM6DSL_FINE,sensor_calib.imu_fine);
  imu.calib_get(LSM6DSL_COARSE,sensor_calib.imu_coarse);
  imu.calib_get(H3LIS331DL,sensor_calib.imu_hi_g);
      
  // Display offsets
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
  SerialUSB.println("Rotate the unit.");

  // Perform calibration
  mag.calibrate(CALIB_SECONDS);  
  mag.calib_get(sensor_calib.mag);

  // Show offsets
  SerialUSB.println("Magnetometer:");
      
  SerialUSB.print(sensor_calib.mag[0]);
  SerialUSB.print(',');
  SerialUSB.print(sensor_calib.mag[1]);
  SerialUSB.print(',');
  SerialUSB.println(sensor_calib.mag[2]);

  SerialUSB.println("All calibration complete. ");

  // Save calibration values
  calib_store.write(sensor_calib);

  SerialUSB.println("Calibration values stored. Please restart the device");

  while(SerialUSB.available())
  {
    char c = SerialUSB.read();
  } 
  while(!SerialUSB.available()){}
  
  // Wait forever
  playInternationale();

  while(1){};

}

#define REST  0
#define A4    440
#define B4    494
#define C4    523    
#define C4S   554
#define D4    587
#define D4S   622
#define E4    659
#define F4S   740
#define G4    784
#define A5    880
#define B5    988
#define C5    1047

#define E     300
#define Q     600
#define QD    900
#define H     1200

void playInternationale(){

  int notes1[35] = {D4,G4,F4S,A5,G4,D4,B4,E4,C4,E4,A5,G4,F4S,E4,D4,C4,B4,REST,D4,G4,F4S,A5,G4,D4,B4,E4,C4,A5,G4,F4S,A5,C5,F4S,G4,REST};
  int times1[35] = {E,QD,E,E,E,E,E,H,QD,E,QD,E,E,E,E,E,H,Q,Q,QD,E,E,E,E,E,H,Q,E,E,Q,Q,Q,Q,H,Q};

  int notes2[35] = {A5,G4,F4S,E4,F4S,G4,E4,F4S,D4,C4S,D4,E4,A4,A5,G4,F4S,REST,REST,A5,A5,F4S,D4,D4,C4S,D4,B5,G4,G4,F4S,E4,F4S,A5,G4,E4,D4};
  int times2[35] = {E,E,H,E,E,E,E,H,Q,E,E,QD,E,QD,E,H,Q,E,E,QD,E,E,E,E,E,H,E,E,E,E,Q,Q,Q,Q,Q};

  int notes3[35] = {REST,B5,G4};
  int times3[35] = {H,E,E,H,QD,E,H,Q,E,E,H,QD,E,H,Q, Q,H,QD,E,H,QD,E,QD,E,Q,Q,H,Q};


  // 1st part
  for (int i=0;i<35;i++){
    if (notes1[i]){
      beep(IO_LS,notes1[i],times1[i]);
    }
    else{
      delay(times1[i]);
    }
    
  }

  // Second part
  for (int i=0;i<35;i++){
    if (notes2[i]){
      beep(IO_LS,notes2[i],times2[i]);
    }
    else{
      delay(times2[i]);
    }
    
  }

}
