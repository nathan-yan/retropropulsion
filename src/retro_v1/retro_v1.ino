#include <Servo.h>
#include <Wire.h>

#include "sensors/MPU9250.cpp"
#include "sensors/BME280.cpp"

#include "control/orientation/quaternion.cpp"
#include "control/orientation/orientation.cpp"

#include "control/angle/angle.cpp"

#include "util/SPIFlash.cpp"
#include "util/tests.cpp"
#include "util/utils.cpp"

enum STAGES { INITIALIZING, PAD_IDLE, FLIGHT, BURNOUT, RETRO, APOGEE, LANDING};

#define RED_LED        6
#define GREEN_LED      12
#define BLUE_LED       10

#define PYRO_ONE     11

#define SLOT_START_ADDRESS      0x186a0

bool ORIENTATION_TESTING = false;
bool SERIAL_DEBUGGING = false;

bool LED_ENABLE = true;

int STAGE_FLAG = INITIALIZING;

/////////////// GUIDANCE ///////////////
// Accel
float ax;
float ay;
float az;

// Gyro
float gx;
float gy;
float gz;

// Averaged gyro rotation rates for derivative term of PID
float avg_gy = 0;   
float avg_gz = 0;

int elapsedTime;
int currentTime;

////////////////// MISC ///////////////////
int c = 0;    // Timesteps
int loopMilliseconds = 20; // Every loop should last `loopMilliseconds` long

int accelFlightThreshold = 11; // How many m/s/s you need to be accelerating in the x-axis to switch to flight mode
int accelFlightHoldTime = 5; // How many iterations you need to hold `accelThreshold` x-axis acceleration
int accelFlightHoldCounter = 0;

uint32_t currentAddress = 0;  // This is the address that is currently AVAILABLE to write to
uint8_t launchSlot = 0;   // Which slot are we recording launch data to?
const int slotSpace = 100000;    // In bytes
const int numSlots = 5;
const int configSize = 28;        // In bytes

// Initialize objects
// Initialize a MPU-9250 object as IMU
MPU9250 IMU(Wire, 0x68);
BME280  BARO;
SPIFlash FLASH(8);

Orientation orientation;

Pyro PYRO_1(PYRO_ONE);

Servo yawServo;
Servo pitchServo;

int flightStartCounter = 0;

void setup() {
  Serial1.begin(9600);      // Begin Bluetooth serial comm
  SerialUSB.begin(9600);   // Begin USB debugging serial comm

  /*
  for (int i = 0; i < 10; i++){
    tone(A4, 300, 100);
    delay(100);  
  }*/
  

  while (!Serial1 || (!SerialUSB && SERIAL_DEBUGGING)) {
    delay(100);   // Wait for bluetooth serial to come up. We don't wait for USB serial because we might not be debugging
  }

  // Attach LED pins
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);   
  pinMode(RED_LED, OUTPUT);

  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, LOW);

  // Change color to red for initialization stage
  // TODO: Make a method for this lol
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);


  Wire.begin(0);

  if (FLASH.initialize()){
    SerialUSB.println("Flash memory unit... OK!");
  } else {
    SerialUSB.println("Flash memory failed to initialize... aborting!");
  }

  // Look through each launch slot to check if the first byte has been written to, starting from address 100,000  
  for (int slot = 0; slot < numSlots; slot++){
    if (FLASH.readByte(0x186a0 + slot * slotSpace) == 0xff){
      launchSlot = slot;
      currentAddress = 0x186a0 + slot * slotSpace;
      break;
    }
  }

  SerialUSB.print("Slot ");
  SerialUSB.print(launchSlot);
  SerialUSB.println(" is available!");

  SerialUSB.println(BARO.beginI2C());
  if (BARO.beginI2C() == true){
    SerialUSB.println("Barometer unit... OK!");
  } else {
    SerialUSB.println("Barometer unit failed to initialize... aborting!");
    abort();
  }

  int res = IMU.begin();
  if (res == 1) {
    SerialUSB.println("Inertial measurement unit... OK!");
    SerialUSB.println(res);
  } else {
    SerialUSB.println("Inertial measurement unit failed to initialize... aborting!");
    SerialUSB.println(res);
    abort();
  }

  // Set the gyroscope to +-500 degrees/sec
  if (IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS)) {
    SerialUSB.println("Restricting gyro to +/- 500 dps");
  } else {
    SerialUSB.println("Failed to restrict gyro range... aborting!");
    abort();
  }

  // Set low-pass filter frequency to 20 hz, so we will sample at a rate of 40 hz
  if (IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ)) {
    SerialUSB.println("Set low pass filter frequency to 20 hz");
  } else {
    SerialUSB.println("Failed to set low pass filter to 20 hz... aborting!");
    abort();
  }

  SerialUSB.println("\nPerforming gyro calibration");
  IMU.calibrateGyro();

  SerialUSB.println("\n\nZenith initialization complete!");

  delay(200);

  // Switch to pad idle mode
  setPadIdleMode();
}

void loop() {
  if (c == 0) {
    elapsedTime = 10;
  } else {
    elapsedTime = millis() - currentTime;
  }

  currentTime = millis();

  // Perform LED blinkies, only if LED is enabled and we're not in test mode
  if (LED_ENABLE){
    if (STAGE_FLAG == PAD_IDLE){
      if (c % 5 == 0) {
          digitalWrite(BLUE_LED, (c / 5) % 2);
      }
    }else if (STAGE_FLAG == FLIGHT){
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, HIGH);
    }else if (STAGE_FLAG == BURNOUT){
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, (c / 5) % 2);
      digitalWrite(BLUE_LED, ((c / 5) + 1) % 2);
    }
  }

  // ORIENTATION BLOCK
  if (c % 1 == 0) {
    IMU.readSensor();

    ax = round((IMU.getAccelX_mss() + 0.1) * 10) / 10.;
    ay = round((IMU.getAccelY_mss() + 0.5) * 10) / 10.;
    az = round((IMU.getAccelZ_mss() - 0.15) * 10) / 10.;

    gx = round(IMU.getGyroX_rads() * 180 / PI * 10) / 10.;
    gy = round(IMU.getGyroY_rads() * 180 / PI * 10) / 10.;
    gz = round(IMU.getGyroZ_rads() * 180 / PI * 10) / 10.;

    avg_gy += gy;
    avg_gz += gz;

    if (STAGE_FLAG == PAD_IDLE){
      // Check x axis acceleration if in pad idle mode
      // We detect flight and save battery by vectoring only when the rocket has launched. Also helps to prevent unwanted movement during setup.
      
      if (ax > accelFlightThreshold){
        accelFlightHoldCounter++;
      } else {
        accelFlightHoldCounter = 0;
      }

      if (accelFlightHoldCounter > accelFlightHoldTime){
        setFlightMode();
      }
    }


    // updateOrientation(gx, gz, gy, az, ay, ax, elapsedTime, 0.1);
    if (STAGE_FLAG == FLIGHT){
      // THIS LINE BELOW IS THE CORRECT ONE FOR REAL FLIGHT
      orientation.updateOrientation(gx, gy, gz, 0, 0, 0, elapsedTime, 0.0);

      /*
      Orientation test bloc, if you want to make a scenario to check behavior, edit it here

      if (c - flightStartCounter < 100){
        orientation.updateOrientation(0, 0, 10, 0, 0, 0, elapsedTime, 0.0);
      }else if (c - flightStartCounter < 200){
       //orientation.updateOrientation(0, 0, -10, 0, 0, 0, elapsedTime, 0.0);
      }else{
        orientation.updateOrientation(100, 0, 0, 0, 0, 0, elapsedTime, 0.0);
      }
      */

    }else{
       orientation.updateOrientation(gx, gy, gz, -ax, -ay, -az, elapsedTime, 0.06);
    }

    orientation.updateYPR();
  }

  if (c % 1 == 0 && ORIENTATION_TESTING) {

    // SerialUSB.println("   ");

    SerialUSB.print(orientation.quaternion.a);
    SerialUSB.print(" ");

    SerialUSB.print(orientation.quaternion.b);
    SerialUSB.print(" ");

    SerialUSB.print(orientation.quaternion.c);
    SerialUSB.print(" ");

    SerialUSB.println(orientation.quaternion.d);
    //SerialUSB.println("");

  }

  if (c % 1 == 0) {
    /*
    SerialUSB.print(ax);
    SerialUSB.print(" ");
    SerialUSB.print(ay);
    SerialUSB.print(" ");
    SerialUSB.print(az);
    SerialUSB.print("   ");


    SerialUSB.print(gx);
    SerialUSB.print(" ");
    SerialUSB.print(gy);
    SerialUSB.print(" ");
    SerialUSB.print(gz);
    SerialUSB.print("   ");
    */
    SerialUSB.print(orientation.yaw);
    SerialUSB.print(" ");
    SerialUSB.print(orientation.pitch);
    SerialUSB.print(" ");
    SerialUSB.print(orientation.roll);
    SerialUSB.print("   ");

    float xyzOrientation[3];

    YZXtoXYZ(xyzOrientation, orientation.yaw, orientation.pitch, orientation.roll);
  
    SerialUSB.print(xyzOrientation[1]);
    SerialUSB.print(" ");
    SerialUSB.print(xyzOrientation[2]);
    SerialUSB.print(" ");
    SerialUSB.print(xyzOrientation[0]);
    SerialUSB.print("   ");

    SerialUSB.print(elapsedTime);
    SerialUSB.print(" ");
    
    SerialUSB.println("");
  }

  if (STAGE_FLAG == FLIGHT){
    float dataToLog[14] = {gx, gy, gz, ax, ay, az, orientation.yaw, orientation.pitch, orientation.roll, 0, 0, elapsedTime};
    uint8_t buffer[23];
    dataToBytes(dataToLog, buffer, 14, 23);

    FLASH.writeBytes(currentAddress, buffer, 23);
    currentAddress += 23; 
  }

  // Increment timesteps
  c += 1;

  // PYRO CHECKS 
  // VERY IMPORTANT THAT THIS LINE IS HERE, OTHERWISE PYROS WILL NEVER TURN OFF
  pyro_check();

  int loopTime = millis() - currentTime; // Time it took for the entire loop to execute
  delay(20 - loopTime);                  // Add extra delay so that it's exactly 20 millis
}


/* HELPER FUNCTIONS */
void setPadIdleMode(){
  STAGE_FLAG = PAD_IDLE;
  resetLED();
}

void setFlightMode(){
  STAGE_FLAG = FLIGHT;

  // Null out roll right before flight starts
  // float roll = orientation.roll;
  // orientation.updateOrientation(-roll, 0, 0, 0, 0, 0, 1000, 0.0);

  SerialUSB.println("FLIGHT STARTED");

  flightStartCounter = c;

  resetLED();
}

void setBurnoutMode(){
  STAGE_FLAG = BURNOUT;

  resetLED();
}

void setRetroMode(){
  STAGE_FLAG = RETRO;

  PYRO_1.fire(1500);
  resetLED();
}

void resetLED(){
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
}

void pyro_check(){
  PYRO_1.check();
}