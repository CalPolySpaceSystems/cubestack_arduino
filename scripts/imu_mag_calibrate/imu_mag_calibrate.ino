
/* External Libraries */
#include <SPI.h>
#include <Wire.h>

/* Sensor Libraries */
#include "imu_triple.h"
#include "lis2mdl.h"

/* Other libraries */
#include "io_utils.h"

/* Pin Definitions */
#define CS_IMU_FINE       2             // SPI Chip Select for the Fine LSM6DSL Accelerometer/Gyroscope
#define CS_IMU_COARSE     3             // SPI Chip Select for the Coarse LSM6DSL Accelerometer/Gyroscope
#define CS_IMU_HI_G       4             // SPI Chip Select for the H3LIS331DL High-G Accelerometer

#define IO_LS             7            // Digital Pin for the loudspeaker

#define CALIB_READS       32
#define CALIB_SECONDS     30

/* Calibration values */
typedef struct {        
  int16_t imu_fine[6];
  int16_t imu_coarse[6];
  int16_t imu_hi_g[3] ;
  int16_t mag[3];
} calib;

calib sensor_calib;

/* Sensor Class Initializations */
imu_triple  imu = imu_triple(CS_IMU_FINE,CS_IMU_COARSE,CS_IMU_HI_G);
lis2mdl     mag = lis2mdl();

void setup() {

  /* Open all communication ports */
  SerialUSB.begin(9600);
  SerialUSB.setTimeout(100000);
  SPI.begin();
  Wire.begin();

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

  beep(IO_LS,660,1000);
  beep(IO_LS,220,1000);
  
  /* Perform calibration if connected to USB */

  while(1){
    if (SerialUSB){
      delay(1000);
      
      /* Magnetometer Calibration */
      // Display instructions
      SerialUSB.println("Rotate the unit.");

      // Perform calibration
      mag.calibrate(CALIB_SECONDS);
      
      mag.calib_get(sensor_calib.mag);

      beep(IO_LS,660,1000);
      
      // Update serial monitor
      SerialUSB.println("Magnetometer:");
      
      SerialUSB.print(sensor_calib.mag[0]);
      SerialUSB.print(',');
      SerialUSB.print(sensor_calib.mag[1]);
      SerialUSB.print(',');
      SerialUSB.println(sensor_calib.mag[2]);

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
      


      /* Beep to indicate completion of calibration + turnoff */
      beep(IO_LS,880,200);
      beep(IO_LS,440,200);
      beep(IO_LS,660,200);

      SerialUSB.println("All calibration complete, please turn off the CubeStack now.");

      break;
    
    }

  }
    
}

void loop() {
  // put your main code here, to run repeatedly:

}
