// Cubestack

/** 
 *  Task Scheduler for CPSS Data Collection
 *
 *  The CPSS data collection and recording system will run using the Arduino TaskScheduler library
 *  
 *  The tasks are listed in the CPSS Data Collection Task Tree.xlsx
 *  
 */
 
 // Includes
#include <TaskScheduler.h>

// Pin Definitions


// Variable Definitions


// Sensor Setup


// Callback methods for Task Scheduler
void accelCallback();
void gyroCallback();
void magCallback();
void baroCallback();
void gpsCallback();
void sendCallback();
void saveCallback();

// Task Definitions
// Task (intervals X 100 ms) for testing
Task pollAccelerometer(500, TASK_FOREVER, &accelCallback);
Task pollGyroscope(500, TASK_FOREVER, &gyroCallback);
Task pollMagnatometer(10000, TASK_FOREVER, &magCallback);
Task pollBarometer(30000, TASK_FOREVER, &baroCallback);
Task pollGPS(50000, TASK_FOREVER, &gpsCallback);
Task sendData(10000, TASK_FOREVER, &sendCallback);
Task saveData(10000, TASK_FOREVER, &saveCallback);

Scheduler runner;

void accelCallback() {
    Serial.print("pollAccelerometer X: ");
    Serial.println(millis());

    Serial.print("pollAccelerometer Y: ");
    Serial.println(millis());

    Serial.print("pollAccelerometer Z: ");
    Serial.println(millis());
}

void gyroCallback() {
    Serial.print("pollGyroscope X: ");
    Serial.println(millis());
    
    Serial.print("pollGyroscope Y: ");
    Serial.println(millis());
    
    Serial.print("pollGyroscope Z: ");
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

void setup () {
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


void loop () {
  runner.execute();
}
