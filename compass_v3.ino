#include <Servo.h>
#include <Wire.h>

#include "sensors/MPU9250.cpp"
#include "sensors/BME280.cpp"

#include "control/orientation/quaternion.cpp"
#include "control/orientation/orientation.cpp"

#include "control/angle/angle.cpp"
#include "control/pid/pid.cpp"

// #include "bluetooth/bluetooth.cpp"
#include "util/SPIFlash.cpp"
#include "util/tests.cpp"
#include "util/utils.cpp"

enum STAGES { INITIALIZING, PAD_IDLE, FLIGHT, LANDING_FLIGHT, BURNOUT, APOGEE, LANDING};

#define RED_LED        6
#define GREEN_LED      12
#define BLUE_LED       10

#define PYRO_ONE       7
#define PYRO_TWO       13
#define PYRO_THREE     11

#define SLOT_START_ADDRESS      0x186a0

bool ENABLE_VECTORING = true;
bool VECTORING_TEST_FLAG = false;
bool ORIENTATION_TESTING = false;
bool SERIAL_DEBUGGING = false;

bool ADD_INTEGRAL = false;

int integral_counter = 0;

bool BLE_CONNECTED = false;
bool BLE_TEST_MODE = false;   // Being in test mode disables default LED blinks, sounds, altitude based pyro and servo control. Everything must be explicitly asked for by user
bool LED_ENABLE = true;

int STAGE_FLAG = INITIALIZING;

////////////////// YAW //////////////////
// Yaw parameters
/*
float yP = 0.38;
float yI = 0.0;
float yD = 0.27;
*/

float yP = 0.13;
float yI = 0.1;
float yD = 0.07;

float yAngle = 0;
float yOffset = 0;

// PID controller for yaw
PIDController yawController(yP, yI, yD, 0);

///////////////// PITCH /////////////////
// Pitch parameters
/*
float pP = 0.38;
float pI = 0.0;
float pD = 0.27;
*/

float pP = 0.13;
float pI = 0.1;
float pD = 0.07;

float pAngle = 0;
float pOffset = 0;

PIDController pitchController(pP, pI, pD, 0);

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

int accelBurnoutThreshold = 6; // How many m/s/s you need to be accelerating in the x-axis to switch to flight mode
int accelBurnoutHoldTime = 5; // How many iterations you need to hold `accelThreshold` x-axis acceleration
int accelBurnoutHoldCounter = 0;

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
Pyro PYRO_2(PYRO_TWO);
Pyro PYRO_3(PYRO_THREE);

Servo yawServo;
Servo pitchServo;

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

  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);

  pinMode(A3, OUTPUT);           // Servo enable pin. This is a way of stopping servo use when Zenith's switch is turned off, so we don't need a high amperage switch
  
  // Always give servos power, but not necessarily control them
  digitalWrite(A3, HIGH);

  // Attach thrust vectoring servos
  yawServo.attach(4);
  pitchServo.attach(3);

  // Perform tests
  //circleFreedomTest(yawServo, pitchServo);
  delay(100);
  yawServo.write(90);
  pitchServo.write(90);
  delay(100);
  //crossFreedomTest(yawServo, pitchServo);
  delay(1000);

  /*for (int i = 0; i < 1000; i++){
    smallAngleTest(yawServo, pitchServo);
    delay(500);
  }*/

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

  // Read through configs to get gimbal offsets
  uint8_t configs[configSize];
  FLASH.readBytes(0xfa0, configs, configSize);

  SerialUSB.println(configs[0]);
  SerialUSB.println(configs[1]);
  SerialUSB.println("OFFSETS");

  if (configs[0] != 0xff){
    yOffset = (configs[0] - 255) / 10.;
  }

  if (configs[1] != 0xff){
    pOffset = (configs[1] - 255) / 10.;
  }

  // Offset sanity checks
  if (abs(yOffset) > 5){
    yOffset = 0;
  }

  if (abs(pOffset) > 5){
    pOffset = 0;
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

  /*
  tone(A4, 3000, 300);
*/

  int res = IMU.begin();
  if (res == 1) {
    SerialUSB.println("Inertial measurement unit... OK!");
    SerialUSB.println(res);
  } else {
    SerialUSB.println("Inertial measurement unit failed to initialize... aborting!");
    SerialUSB.println(res);
    abort();
  }

  // tone(A4, 3000, 300);

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

  /*tone(A4, 300, 30);
  delay(30);
  tone(A4, 1000, 100);
  delay(100);
  tone(A4, 3000, 300);*/

  // Lock servos to 0 position
  gimbalPitchServo(0);
  gimbalYawServo(0);

  delay(200);

  // Switch to pad idle mode
  setPadIdleMode();
}

void loop() {
  if (c % 10 == 0){
    if (SerialUSB.available()){
      String command = "";
      while (SerialUSB.available()){
        command += (char) SerialUSB.read();
      }

      if (command.equals("ERASE_FLASH\n")){
        SerialUSB.println("ERASING FLASH");
        uint8_t configValues[configSize];
        FLASH.readBytes(0xfa0, configValues, configSize);

        FLASH.chipErase();
        FLASH.writeBytes(0xfa0, configValues, configSize);

        SerialUSB.println("DONE ERASING FLASH");

      }else if (command.equals("DUMP_SLOT_0\n")){
        SerialUSB.println("DUMPING SLOT 0");
        int counter = 0;
        while (counter < 30000) {     // 30,000 / 690 = roughly 30 seconds of data
          SerialUSB.print(FLASH.readByte(0x186a0 + 0 * slotSpace + counter), HEX);
          SerialUSB.print(' ');

          if (counter % 100 == 0){
            SerialUSB.println("");
          }

          counter++;
        }
      }else if (command.equals("DUMP_SLOT_1\n")){
        SerialUSB.println("DUMPING SLOT 1");
        int counter = 0;
        while (counter < 50000) {     // 30,000 / 690 = roughly 30 seconds of data
          SerialUSB.print(FLASH.readByte(0x186a0 + 1 * slotSpace + counter), HEX);
          SerialUSB.print(' ');

          if (counter % 100 == 0){
            SerialUSB.println("");
          }

          counter++;
        }
      }else if (command.equals("DUMP_SLOT_2\n")){
        SerialUSB.println("DUMPING SLOT 2");
        int counter = 0;
        while (counter < 50000) {     // 30,000 / 690 = roughly 30 seconds of data
          SerialUSB.print(FLASH.readByte(0x186a0 + 2 * slotSpace + counter), HEX);
          SerialUSB.print(' ');

          if (counter % 100 == 0){
            SerialUSB.println("");
          }

          counter++;
        }
      }else if (command.equals("DUMP_SLOT_3\n")){
        SerialUSB.println("DUMPING SLOT 3");
        int counter = 0;
        while (counter < 30000) {     // 30,000 / 690 = roughly 30 seconds of data
          SerialUSB.print(FLASH.readByte(0x186a0 + 3 * slotSpace + counter), HEX);
          SerialUSB.print(' ');

          if (counter % 100 == 0){
            SerialUSB.println("");
          }

          counter++;
        }
      }else if (command.equals("DUMP_SLOT_4\n")){
        SerialUSB.println("DUMPING SLOT 4");
        int counter = 0;
        while (counter < 30000) {     // 30,000 / 690 = roughly 30 seconds of data
          SerialUSB.print(FLASH.readByte(0x186a0 + 4 * slotSpace + counter), HEX);
          SerialUSB.print(' ');

          if (counter % 100 == 0){
            SerialUSB.println("");
          }

          counter++;
        }
      }
    }
  }

  if (c != 0 and c % 4 == 0) {   // Bluetooth sends and receives at a rate of 1000 / (20 * 4) Hz = 12.5 Hz, we don't want BLE to slow down the rest of the program
    // Report current gyroscope readings and orientation.
    // Because of limits of BLE packet sizes, we have to stagger the sending of the state.

    int part = ((c / 4) % 3);

    char payload[20];   // Create a char buffer to write to Serial

    String orientationPayload = BLEGenerateOrientationPayload(part);
    orientationPayload.toCharArray(payload, 20);

    if (BLE_CONNECTED){
      Serial1.write(payload);
    }

    // Check for any signals from an app
    if (Serial1.available()) {
      String command = "";
      int numBytes = 0;

      // Limit the size of a command to 10 bytes
      while (Serial1.available() && numBytes < 10) {
        command += (char) Serial1.read();
      }
      SerialUSB.println(command);

      int8_t length = command.length();

      if (command.startsWith("INC_YAW")){
        String incrementAmountString = command.substring(8);
        float incrementAmount = incrementAmountString.toFloat();
        yOffset += incrementAmount;

        gimbalYawServo(0);
      } else if (command.startsWith("INC_PITCH")){
        String incrementAmountString = command.substring(10);
        float incrementAmount = incrementAmountString.toFloat();
        pOffset += incrementAmount;

        gimbalPitchServo(0);
      } else if (command.equals("SAVE_OFFSETS")){
        SerialUSB.println("SAVING OFFSETS");
        uint8_t newOffsets[2] = {(int) clip(yOffset * 10 + 255, 0, 255), (int) clip(pOffset * 10 + 255, 0, 255)};
        uint8_t addresses[2]  = {0x00, 0x01};
        editConfigs(FLASH, 28, newOffsets, addresses, 2);
      }

      if (command.startsWith("LED")) {
        if (command.equals("LED_DISABLE")) {
          LED_ENABLE = false;
        } else if (command.equals("LED_ENABLE")){
          LED_ENABLE = true;
        } else if (command.startsWith("LED_ON")){
          char ledIdentifier = command.charAt(length - 1);
          SerialUSB.println(ledIdentifier);
          if (ledIdentifier == 'R'){
            digitalWrite(RED_LED, LOW);
          } else if (ledIdentifier == 'G'){
            digitalWrite(GREEN_LED, LOW);
          } else if (ledIdentifier == 'B'){
            digitalWrite(BLUE_LED, LOW);
          }
        } else if (command.startsWith("LED_OFF")){
          char ledIdentifier = command.charAt(length - 1);
          SerialUSB.println(ledIdentifier);
          if (ledIdentifier == 'R'){
            digitalWrite(RED_LED, HIGH);
          } else if (ledIdentifier == 'G'){
            digitalWrite(GREEN_LED, HIGH);
          } else if (ledIdentifier == 'B'){
            digitalWrite(BLUE_LED, HIGH);
          }
        }
       
      } else if (command.startsWith("PYRO_ON")) {
        if (command.endsWith("1")) {
          PYRO_1.fire(1000);
        } else if (command.endsWith("2")) {
          PYRO_2.fire(1000);
        }
      } else if (command.equals("hs_st")) {
        Serial1.write("hs_resp");
        BLE_CONNECTED = true;
      } else if (command.equals("ENTER_TEST_MODE")){
        enterBLETestMode();
      } else if (command.equals("EXIT_TEST_MODE")) {
        exitBLETestMode();
      } else if (command.equals("TVC_TEST_CROSS")){
        crossFreedomTest(yawServo, pitchServo);
      } else if (command.equals("GET_SLOT_STATES")){
        String states = "SST_";
        for (int slot = 0; slot < numSlots; slot++){
          if (FLASH.readByte(0x186a0 + slot * slotSpace) == 0xff){
            states += "1";
          }else{
            states += "0";
          }
      } 

        states += launchSlot;

        SerialUSB.println(states);

        char payload[20];   // Create a char buffer to write to Serial

        states.toCharArray(payload, 20);
        delay(30);
        Serial1.write(payload);

      }else if (command.startsWith("SET_SLOT")){
        char slot = command.charAt(length - 1);
        int slotNumber = slot - 48;

        launchSlot = slotNumber;
        currentAddress = 0x186a0 + launchSlot * slotSpace;

      }
    }
  }

  if (c == 0) {
    elapsedTime = 10;
  } else {
    elapsedTime = millis() - currentTime;
  }

  currentTime = millis();

  // Perform LED blinkies, only if LED is enabled and we're not in test mode
  if (LED_ENABLE && !BLE_TEST_MODE){
    if (STAGE_FLAG == PAD_IDLE){
      if (c % 5 == 0) {
          digitalWrite(BLUE_LED, (c / 5) % 2);
      }
    }else if (STAGE_FLAG == FLIGHT){
      if (ADD_INTEGRAL){
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, LOW);
      }else{
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(RED_LED, HIGH);
      }
    }else if (STAGE_FLAG == BURNOUT){
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, (c / 5) % 2);
      digitalWrite(BLUE_LED, ((c / 5) + 1) % 2);
    }
  }

  /*if (STAGE_FLAG == FLIGHT){
    integral_counter++;

    if (integral_counter > 500){
      ADD_INTEGRAL = true;

      if (integral_counter == 501){
        yawController.reset();
        pitchController.reset();
      }

      yawController.setParameters(.13, 0.5, 0.07);
      pitchController.setParameters(.13, 0.5, 0.07);
    }

  }*/
  
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

    if (STAGE_FLAG == FLIGHT){
      // Check acceleration for free fall
      if (ax * ax + ay * ay + az * az < accelBurnoutThreshold){
        accelBurnoutHoldCounter++;
      }else{
        accelBurnoutHoldCounter = 0;
      }

      if (accelBurnoutHoldCounter > accelBurnoutHoldTime){
        setBurnoutMode();
      }
    }

    // updateOrientation(gx, gz, gy, az, ay, ax, elapsedTime, 0.1);
    if (STAGE_FLAG == FLIGHT){
      orientation.updateOrientation(-gz, gy, gx, az, -ay, -ax, elapsedTime, 0.0);   // Don't fuse accelerometer data
    }else{
      orientation.updateOrientation(-gz, gy, gx, az, -ay, -ax, elapsedTime, 0.02);
    }

    orientation.updateYPR();
  }

  // VECTORING BLOCK
  // Vectoring runs at 1000 / (20 * 3) = 16.6 Hz
  if (c % 3 == 0) {
    // Only vector if we're in the flight stage, or if we're testing vectoring, and ensure that we aren't currently in bluetooth test mode
    if ((STAGE_FLAG == FLIGHT or VECTORING_TEST_FLAG) && !BLE_TEST_MODE) {

      pAngle = pitchController.step(orientation.pitch, avg_gy / 3.);
      avg_gy = 0;

      // Convert the gimbal angle to a direct servo angle

      yAngle = yawController.step(orientation.yaw, -avg_gz / 3.);
      avg_gz = 0;

      // Place pAngle and yAngle in yaw-pitch-roll 
      // Then convert into roll-yaw-pitch
      // theta1 = yaw
      // theta2 = pitch 
      // theta3 =
      /*
      pAngle = -pAngle;
      yAngle = -yAngle;

      pAngle = clip(pAngle, -30, 30);
      yAngle = clip(yAngle, -30, 30);
      
      float m13 = cos(yAngle * PI / 180.) * sin(pAngle * PI / 180.) * sin(orientation.roll * PI / 180.) + sin(yAngle * PI / 180.) * cos(orientation.roll * PI / 180.);
      float m12 = -cos(yAngle * PI / 180.) * sin(pAngle * PI / 180.) * cos(orientation.roll * PI / 180.) + sin(yAngle * PI / 180.) * sin(orientation.roll * PI / 180.);
      float m11 =cos(yAngle * PI / 180.) * cos(pAngle * PI / 180.);

      pAngle = atan2(m13, sqrt(1 - m13 * m13)) * 180. / PI;
      yAngle = atan2(-m12, m11) * 180. / PI;
      */

      // Find angle with largest magnitude, this is the one we want to shrink
      float biggestAngle = max(abs(pAngle), abs(yAngle));

      // Find ratio, and proportionally shrink yaw and pitch so that they're within 8 degrees while maintaining (approximately) the direction as well
      if (biggestAngle > 8){
        float ratio = abs(8. / biggestAngle);
        pAngle *= ratio;
        yAngle *= ratio;
      }

      gimbalYawServo(-yAngle);
      gimbalPitchServo(-pAngle);
    }
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

    SerialUSB.print(orientation.yaw);
    SerialUSB.print(" ");
    SerialUSB.print(orientation.pitch);
    SerialUSB.print(" ");
    SerialUSB.print(orientation.roll);
    SerialUSB.print("   ");

    SerialUSB.print(-pAngle);
    SerialUSB.print(" ");
    SerialUSB.print(yAngle);
    SerialUSB.print(" ");

    SerialUSB.print(elapsedTime);
    SerialUSB.print(" ");

    SerialUSB.println("");
  }

  if (STAGE_FLAG == FLIGHT){
    float dataToLog[14] = {gx, gy, gz, ax, ay, az, orientation.yaw, orientation.pitch, orientation.roll, pAngle, yAngle, elapsedTime};
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
static inline void writeToServo(Servo servo, int angle, bool output) {
  if (!ENABLE_VECTORING) {
    return;
  }

  if (output) {
    SerialUSB.print(angle);
    SerialUSB.print(" ");
  }
  servo.write(angle);
  // servo.detach();
}

void gimbalYawServo(float gimbalAngle){
  float yServoAngle = linkage2(toplink_startAngle + gimbalAngle + yOffset) + 180;

  writeToServo(yawServo, yServoAngle, false);
}

void gimbalPitchServo(float gimbalAngle){
  float pServoAngle = linkage(bottomlink_startAngle + gimbalAngle + pOffset);

  writeToServo(pitchServo, pServoAngle, false);
}

void pyro_check(){
  PYRO_1.check();
  PYRO_2.check();
  PYRO_3.check();
}

// bruh make this use the dataToBinary function in utils.cpp, then we don't need to stagger data sending, cuz all 117 bits of orientation data can fit in 20 bytes
String BLEGenerateOrientationPayload(int part) {
  String payload;

  switch (part)
  {
    case 0:
      payload = "A";
      payload += ax;
      payload += " ";
      payload += ay;
      payload += " ";
      payload += az;
      break;

    case 1:
      payload = "G";
      payload += gx;
      payload += " ";
      payload += gy;
      payload += " ";
      payload += gz;

      break;

    case 2:
      payload = "O";
      payload += orientation.yaw; 
      payload += " ";
      payload += orientation.pitch;
      payload += " ";
      payload += orientation.roll;

      break;

    default:
      break;
  }

  return payload;
}

void setPadIdleMode(){
  STAGE_FLAG = PAD_IDLE;
  resetLED();
}

void setFlightMode(){
  STAGE_FLAG = FLIGHT;

  yawController.start();
  pitchController.start();

  resetLED();
}

void setBurnoutMode(){
  STAGE_FLAG = BURNOUT;

  // Fire pyro 2
  PYRO_3.fire(1500);

  resetLED();
}

void enterBLETestMode(){
  BLE_TEST_MODE = true;
  resetLED();
}

void exitBLETestMode(){
  BLE_TEST_MODE = false;
  resetLED();
}

void resetLED(){
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
}

void changeSetpoints(float yaw, float pitch){
  yawController.changeSetpoint(yaw);
  pitchController.changeSetpoint(pitch);
}

void setGuidanceProgram(float yaw, float yawSpeed, float pitch, float pitchSpeed){
  // yawController.beginProgram(yaw, yawSpeed);
  // pitchController.beginProgram(pitch, yawSpeed);
}