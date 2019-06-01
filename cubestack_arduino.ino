// Cubestack

/** 
 *  Task Scheduler for CPSS Data Collection
 *
 *  The CPSS data collection and recording system will run using the Arduino TaskScheduler library
 *  
 *  The tasks are listed in the CPSS Data Collection Task Tree.xlsx
 *  
 */

#define VERSION "0.9.1"

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
#include "src/sensor/ptap_ads1115.h"

/* Other libraries */
#include "src/util/io_utils.h"
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

/* Other operational definitions */
#define CALIB_READS       32
#define CALIB_SECONDS     45
#define CALIB_TIMEOUT     10

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
#define TAP_ID          142

#define END             0x4350

#define MODE_ID         0xA5
#define FLASH_ID        0xB4

#define STAT            0x70
#define STAT_LAUNCH     0x01
#define STAT_CALIB      0x02
#define STAT_FL_EMPTY   0x04
#define STAT_REC        0x08

bool dataMode = 0;
bool calibrated = 0;
bool flashData = 0;
bool recording = 0; 

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

#define TAP_FAST        7500
#define TAP_SLOW        15000

#define STATE_FAST      19224
#define STATE_SLOW      STATE_FAST*8

#define STATUS_INTERVAL 1000000
#define CMD_INTERVAL    1000000

/* Sensor Class Initializations */
imu_triple      imu = imu_triple(CS_IMU_FINE,CS_IMU_COARSE,CS_IMU_HI_G);
lis2mdl         mag = lis2mdl();
SFE_UBLOX_GPS   gps;

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

float rpy[3] = {0.0, 0.0, 0.0};

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
uint32_t tap_micros;
uint32_t gps_micros;
uint32_t state_micros;

/* Prototype callback methods for Task Scheduler */
void imuPoll();
void magPoll();
void baroPrimeTemp();
void baroPollTemp();
void baroPollPress();
void gpsPoll();
void ptapPoll1();
void ptapPoll2();
void ptapPoll3();

#ifdef MODE_STATE
void margCallback();
#endif

/* Telemetry Callbacks */
#ifdef MODE_COATS
void imuDownlink();
void baroDownlink();
void magDownlink();
void gpsDownlink();
void tapDownlink();
void stateDownlink();
void statusDownlink();
void cmdUplink();
void deassertRec();

/* Command Callbacks */
void setMode(uint8_t param);
void eraseFlash(uint8_t param);

#endif

void deassertRec();

/* Polling Task Definitions */
Task pollImu(IMU_SLOW, TASK_FOREVER, &imuPoll);
Task pollMag(MAG_SLOW, TASK_FOREVER, &magPoll);
Task pollBaro(BARO_GPS_SLOW, TASK_FOREVER, &baroPrimeTemp);
Task pollTap(TAP_FAST, TASK_FOREVER, &ptapPoll1);
Task pollGPS(BARO_GPS_SLOW, TASK_FOREVER, &gpsPoll);

/* State Estimator Task Definition */
#ifdef MODE_STATE
Task margEst(STATE_FAST, TASK_FOREVER, &margCallback);
#endif

/* Telemetry Task Definitions */
#ifdef MODE_COATS
Task tlmImu(IMU_SLOW, TASK_FOREVER, &imuDownlink);
Task tlmBaro(BARO_GPS_SLOW, TASK_FOREVER, &baroDownlink);
Task tlmMag(MAG_SLOW, TASK_FOREVER, &magDownlink);
Task tlmGPS(BARO_GPS_SLOW, TASK_FOREVER, &gpsDownlink);
Task tlmTap(TAP_SLOW ,TASK_FOREVER, &tapDownlink);
Task cmdRx(CMD_INTERVAL, TASK_FOREVER, &cmdUplink);
Task tlmStatus(STATUS_INTERVAL, TASK_FOREVER, &statusDownlink);
Task tlmState(STATE_SLOW,TASK_FOREVER, &stateDownlink);
#endif

/* Other Task Definitions */
Task deassertRecBtn(500000,1,&deassertRec);

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

  digitalWrite(IO_LS,LOW);

  #ifdef MODE_FLASH

  flash.begin();

  // Check for data in flash
  for (uint8_t i=0;i<16;i++){
    char b = flash.readByte(i);
    if ((uint8_t)b != 0xFF){

      beep(IO_LS,523,400);
      beep(IO_LS,554,200);
      beep(IO_LS,494,400);
      
      // Do not allow flash to be unlocked
      flashData = 1;
 
      break;
      
    }
  }
  
  if (!flashData){
    
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

      printHeader();

      #ifdef MODE_FLASH
      
      flashOffload(flashData);

      #endif

      for (int j = 0; j<6;j++){
        if(sensor_calib.imu_fine[i]){
          calibrated = true;
        } 
      }
      
      calibrate(calibrated);
    }

    delay(1000);
  
  }

  beep(IO_LS,523,600);
  beep(IO_LS,1046,600);

  delay(300);
  
  /* Notify if uncalibrated */ 
  if(!calibrated){
    
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
  downlink.addTlm(TAP_ID,&p_data,sizeof(p_data));

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
  runner.addTask(pollTap);
  runner.addTask(pollGPS);

  #ifdef MODE_STATE
  runner.addTask(margEst);
  #endif
  
  #ifdef MODE_COATS
  runner.addTask(tlmImu);
  runner.addTask(tlmMag);
  runner.addTask(tlmBaro); 
  runner.addTask(tlmGPS);
  runner.addTask(tlmTap);
  runner.addTask(cmdRx);
  runner.addTask(deassertRecBtn);
  runner.addTask(tlmStatus);
  #endif 

  /* Enable Tasks */
  pollImu.enable();
  pollMag.enable();
  pollBaro.enable();
  pollTap.enable();
  pollGPS.enable();

  #ifdef MODE_STATE
  margEst.enable();
  #endif

  #ifdef MODE_COATS
  tlmImu.enable();
  tlmMag.enable();
  tlmBaro.enable();
  tlmGPS.enable();
  tlmTap.enable();
  cmdRx.enable();
  tlmStatus.enable();

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

/* Pressure Taps */
void ptapPoll1() {

  tap_micros = micros();

  p_data[0] = readRaw_ads1115(ADC_A_ADDR);
  prime_ads1115(ADC_A_ADDR, 1); //A at ch1

  p_data[1] = readRaw_ads1115(ADC_B_ADDR);
  prime_ads1115(ADC_B_ADDR, 1); //B at ch1

  pollTap.setCallback(&ptapPoll2);
  pollTap.delay(1000);
}

void ptapPoll2() {
  p_data[2] = readRaw_ads1115(ADC_A_ADDR);
  prime_ads1115(ADC_A_ADDR, 2); //A at ch2

  p_data[3] = readRaw_ads1115(ADC_B_ADDR);
  prime_ads1115(ADC_B_ADDR, 2); //B at ch2

  pollTap.setCallback(&ptapPoll3);
  pollTap.delay(1000); //calculated to be 983 microseconds, round to 1000
}

void ptapPoll3() {

  p_data[4] = readRaw_ads1115(ADC_A_ADDR);
  prime_ads1115(ADC_A_ADDR, 0); //A at ch0

  p_data[5] = readRaw_ads1115(ADC_B_ADDR);
  prime_ads1115(ADC_B_ADDR, 0); //B at ch0

  // Store data in flash
  #ifdef MODE_FLASH
  if (flashWrite){
    uint8_t buf[32];
    uint8_t sz;

    sz = downlink.buildTlm(TAP_ID,buf,tap_micros);
    flash.writeByteArray(nextAddress,buf,sz,false);
    nextAddress+=(sz);
  }
  
  #endif
    
  pollTap.setCallback(&ptapPoll1);

}

/*_______________State Estimation Callbacks______________*/

#ifdef MODE_STATE
void margCallback() {

  state_micros = micros();

  // Update filter and calculate attitude
  filter_update(q,&i_flt,&i_raw,&m_flt);

  // Convert quaternians to roll/pitch/yaw
  //conv_q_rpy(q,rpy);

  #ifdef MODE_DEBUG
  
  // Print all data
  //SerialUSB.print(est_micros);
  //SerialUSB.print(',');
  //SerialUSB.print(rpy[0]);
  //SerialUSB.print(',');
  //SerialUSB.print(rpy[1]);
  //SerialUSB.print(',');
  //SerialUSB.println(rpy[2]);
  #endif

  #ifdef MODE_FLASH
  if (flashWrite){
    uint8_t buf[32];
    uint8_t sz;

    sz = downlink.buildTlm(STATE_ID,buf,state_micros);
    flash.writeByteArray(nextAddress,buf,sz,false);
    nextAddress+=(sz);
  }
  #endif
  
}
#endif


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


void tapDownlink(){
  
  downlink.serialWriteTlm(SerialDownlink,TAP_ID,tap_micros);
  
}

#ifdef MODE_STATE
void stateDownlink(){
  
  downlink.serialWriteTlm(SerialDownlink,STATE_ID,state_micros);
  
}
#endif

void cmdUplink(){

  downlink.serialParseCmd(SerialDownlink);
  
}

void statusDownlink(){
  
  uint8_t statusCurrent = STAT | (dataMode) | (calibrated << 1) | (flashData << 2) | (recording << 3);

  if (nextAddress>=8000000){
    flashWrite = false;
  }
  
  downlink.serialWriteStat(SerialDownlink,statusCurrent);
  
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
      pollTap.setInterval(TAP_SLOW);

      #ifdef MODE_COATS
      tlmImu.setInterval(IMU_SLOW);
      tlmBaro.setInterval(BARO_GPS_SLOW);
      tlmGPS.setInterval(BARO_GPS_SLOW);
      tlmTap.setInterval(TAP_SLOW*2);
      
      #endif

      #ifdef MODE_FLASH

      flashWrite = false;

      #endif

      dataMode = 0;

      if (recording){

        // Toggle IO
        digitalWrite(IO_REC,HIGH);
        deassertRecBtn.restartDelayed(100000);
        recording = 0;
        
      }
      
      break;

    // Launch
    case 0x11:
      
      pollImu.setInterval(IMU_FAST);
      pollMag.setInterval(MAG_FAST);
      pollBaro.setInterval(BARO_GPS_FAST);
      pollGPS.setInterval(BARO_GPS_FAST);
      pollTap.setInterval(TAP_FAST);

      #ifdef MODE_COATS
      tlmImu.setInterval(IMU_FAST);
      tlmBaro.setInterval(BARO_GPS_SLOW);
      tlmGPS.setInterval(BARO_GPS_SLOW);
      tlmTap.setInterval(TAP_SLOW);

      #endif

      #ifdef MODE_FLASH

      flashWrite = true;
      flashData = 1;
  
      #endif

      dataMode = 1;

      if (!recording){

        // Toggle IO
        digitalWrite(IO_REC,HIGH);
        deassertRecBtn.restartDelayed(100000);
        recording = 1;
        
      }

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

    flashData = 0;
    nextAddress = 0;

  #endif

  }
  
}

/*____________________Other Functions__________________*/

void deassertRec(){

  // Toggle IO
  digitalWrite(IO_REC,LOW);

  // Disable the task
  deassertRecBtn.disable();
  
}

/*_______________Initialization Functions_______________*/

void printHeader(){
  
  SerialUSB.println("______________________________________");
  SerialUSB.println("|       CAL POLY SPACE SYSTEMS       |");
  SerialUSB.println("|    CUBESTACK TELEMETRY SOFTWARE    |");
  
  SerialUSB.print("|           VERSION ");
  SerialUSB.print(VERSION);
  SerialUSB.println("            |");
  SerialUSB.println("|                                    |");
  SerialUSB.println("|-------------{  MMXIX  }------------|");
  SerialUSB.println("|____________________________________|");
  SerialUSB.println("|             WRITTEN BY:            |");
  SerialUSB.println("|           PATRICK CHIZEK           |");
  SerialUSB.println("|              BEN CLARK             |");
  SerialUSB.println("|             ERIC ASHLEY            |");
  SerialUSB.println("|            BRETT GLIDDEN           |");
  SerialUSB.println("|____________________________________|");
  SerialUSB.println("|         You traded emotion         |");
  SerialUSB.println("|    for skills and computerized     |");
  SerialUSB.println("|____________________________________|");
  SerialUSB.println("");
}

#ifdef MODE_FLASH

void flashOffload(bool flashData){

  if(flashData){

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

  else{
    SerialUSB.println("No flash data detected.");
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

  beep(IO_LS,523,200);
  beep(IO_LS,659,400);

  while(SerialUSB.available())
  {
    char c = SerialUSB.read();
  } 
  while(!SerialUSB.available()){}
  
  // Wait forever
  while(1){};

}
