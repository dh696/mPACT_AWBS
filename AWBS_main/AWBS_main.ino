// Code for mPACT Automatic Wake Bar System (AWBS)
// ESP32 WROOM-32 microcontroller
// By Daniel Harrop, 2026


//TODO:
// stallguard for safety.
// stop command - add a new command type, and change MoveCommand structure to take a command type rather than just bool home. Make sure to cancel any currently running commands.
// Could add error checking for commands not completed after certain timeout?


//#include "Arduino.h"
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Preferences.h>
#include "CommandParser2.h" // modified version that accepts optional arguments.
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
//#include "esp_system.h"

// this is for VSCode intellisense, may need to remove for compilation:
size_t strlcpy (char *, const char *, size_t);
size_t strlcat (char*, const char *, size_t);

// For Adafruit TMC2209 breakout
#define R_SENSE 0.05f

class StepperDriver;

// UART 2 to all three stepper drivers
HardwareSerial & stepperSerial = Serial2;
const long SERIAL_BAUD_RATE = 115200;
const int RX_PIN = 16;
const int TX_PIN = 17;

// Preferences instance for storing home positions permanently in non-volatile memory.
Preferences preferences;

// ADC calibration structure:
esp_adc_cal_characteristics_t adc_chars;

// Stepper driver Enable Pin:
const gpio_num_t STEPPER_EN_PIN = GPIO_NUM_4;

// Power supply test input pin: (20V supply connected through 15:1 potential divider)
// This pin is connected to ADC1 channel 0;
const gpio_num_t PSU_TEST_PIN = GPIO_NUM_36;
const adc1_channel_t PSU_TEST_ADC_CHANNEL = ADC1_CHANNEL_0;

// For checking if PSU is on:
const float PSU_VOLTAGE_LOWER = 15.0f;  // ideally this would be closer to 20V but sometimes dips below 19V causing reboot.
const float PSU_VOLTAGE_UPPER = 18.0f;  // upper bound for hysteresis when psu turns back on.

enum MotorID{
  MOTOR_NONE = 0,
  MOTOR_ROT,
  MOTOR_LIN,
  MOTOR_LIN1,
  MOTOR_LIN2,
  MOTOR_ALL
};
char MOTOR_NAMES[6][5] = {
  "NONE",
  "ROT ",
  "LIN ",
  "LIN1",
  "LIN2",
  "ALL "
};

char MOTOR_ARGS[6][5] = {
  "none",
  "rot",
  "lin",
  "lin1",
  "lin2",
  "all"
};

char MOTOR_UNITS[6][4]{
  "",
  "deg",
  "mm",
  "mm",
  "mm",
  ""
};

enum HomeState{
  HOME_IDLE,
  HOME_START,
  HOME_SEARCH,
  HOME_BACKOFF,
  HOME_LATCH,
  HOME_OFFSET,
  HOME_COMPLETE
};


// Move command structure:
struct MoveCommand{
  MotorID motorID {};
  float targetPos {};  // in microsteps
  float speed {};  // in microsteps/s
  float acceleration {}; // in microsteps/s^2
  bool home {false};
};

struct ReturnMessage{
  MotorID motorID {};
  long pos {0};  // in microsteps
  bool home {false};
};

struct ActiveCommand{
  uint8_t remainingReturns {0};
  MoveCommand moveCommand {};
  ReturnMessage waitingForMsgs[3] {};
};

enum ErrorState{
  WARNING,  // not critical, instruction ignored.
  FAILURE  // motors disabled, must be rebooted.
};

struct ErrorMessage{
  char msg[64];
  ErrorState state;
};

// stepper motor current state
enum CmdType{
  NONE,
  MOVE,
  HOME
};


////// MOTOR PARAMETERS ////////////////////////////////////////////////////////////////////////////

struct StepperHardConfig{
  // Motor ID and serial address:
  MotorID motorID;
  uint8_t uartAddr;
  float Rsense;
  // Pins:
  gpio_num_t stepPin;
  gpio_num_t dirPin;
  gpio_num_t diagPin;
  gpio_num_t swPin;
};

struct StepperSoftConfig{
  uint16_t microsteps;
  uint16_t stepsPerUnitPos;
  float travel; // in units
  float maxSpeed; // in units/s
  float maxAccel; // in units/s^2
  float homeSearchSpeed;  // in units/s
  float homeLatchSpeed; // in units/s
  float homeAccel;  // in units/s^2
  float homeMaxDist;  // in units
  float homeBackoff;  // in units, also used for minimum valid position.
  float defaultHomeOffset; // in units - replaced with value stored in EEPROM once set first time.
  float defaultSpeed; // in units/s
  float defaultAcceleration;  // in units/s^2
};

// Nema17 rotation motor:
StepperHardConfig rotHardConfig = {
  .motorID = MOTOR_ROT,
  .uartAddr = 0,
  .Rsense = R_SENSE,
  .stepPin = GPIO_NUM_12,
  .dirPin = GPIO_NUM_25,
  .diagPin = GPIO_NUM_32,
  .swPin = GPIO_NUM_39
};
// Linear actuator 1
StepperHardConfig lin1HardConfig = {
  .motorID = MOTOR_LIN1,
  .uartAddr = 1,
  .Rsense = R_SENSE,
  .stepPin = GPIO_NUM_13,
  .dirPin = GPIO_NUM_26,
  .diagPin = GPIO_NUM_15,
  .swPin = GPIO_NUM_34
};
// linear actuator 2
StepperHardConfig lin2HardConfig = {
  .motorID = MOTOR_LIN2,
  .uartAddr = 2,
  .Rsense = R_SENSE,
  .stepPin = GPIO_NUM_14,
  .dirPin = GPIO_NUM_27,
  .diagPin = GPIO_NUM_2,
  .swPin = GPIO_NUM_35
};

// Nema17 rotation motor:
// seems to be okay going at 100deg/s, with 200deg/s/s acceleration, but slower would allow lower current/less likely to lose steps
// 1.8deg/step Nema17 motor (200 steps per motor rotation, 9:1 gear ratio to turntable):
StepperSoftConfig rotSoftConfig = {
  .microsteps = 4,
  .stepsPerUnitPos = 200*9/360, // full steps per turntable degree
  .travel = 100.0,  // deg from home
  .maxSpeed = 100.0,  // deg/s
  .maxAccel = 200.0,  // deg/s^2
  .homeSearchSpeed = 20.0,  // deg/s
  .homeLatchSpeed = 1.0,  // deg/s
  .homeAccel = 200.0, // deg/s^2
  .homeMaxDist = 105.0, // deg
  .homeBackoff = 2.0,  // deg
  .defaultHomeOffset = 5.0, // deg
  .defaultSpeed = 50.0, // deg/s
  .defaultAcceleration = 100.0 // deg/s^2
};

// Linear actuators
// 18deg/step linear actuators:
// listing quotes 18deg per step, 0.5mm thread pitch, 0.025mm per step, max speed 25mm/s.
StepperSoftConfig linSoftConfig = {
  .microsteps = 4,
  .stepsPerUnitPos = 40, // full steps per mm carriage travel
  .travel = 65.0,  // mm from home, might be able to increase this a little
  .maxSpeed = 15.0,  // mm/s
  .maxAccel = 100.0,  // mm/s^2
  .homeSearchSpeed = 10.0,  // mm/s
  .homeLatchSpeed = 1.0,  // mm/s
  .homeAccel = 100.0, // mm/s^2
  .homeMaxDist = 70.0, //mm
  .homeBackoff = 3.0,  // mm
  .defaultHomeOffset = 37.0,  // mm
  .defaultSpeed = 10.0, // mm/s
  .defaultAcceleration = 50.0 // mm/s^2
};

// Global program variables:

// Flags:
volatile bool psuOn = false;
volatile float psuVoltage = 0.0f;
volatile bool failed = false;
volatile bool waitingForPower = false;
char failureMsg[128] = "";
char incomingChar;
int charCount = 0;
char line[128];
ActiveCommand currentCommands[3];

// Command parser setup:
// CommandParser<COMMANDS, COMMAND_ARGS, COMMAND_NAME_LENGTH, COMMAND_ARG_SIZE, RESPONSE_SIZE>
typedef CommandParser<16,8,10,32,128> cmdParser;
cmdParser parser;
const char ARG_ROT[] = "rot";
const char ARG_LIN[] = "lin";
const char ARG_LIN1[] = "lin1";
const char ARG_LIN2[] = "lin2";
const char ARG_ALL[] = "all";


// Global stepper command queue:
QueueHandle_t motionQueue;
QueueHandle_t returnQueue;
QueueHandle_t errorQueue;

void feedTheDog(uint8_t dog){
  if (dog == 0){
  // feed dog 0
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  TIMERG0.wdt_feed=1;                       // feed dog
  TIMERG0.wdt_wprotect=0;                   // write protect
  } else if (dog == 1){
  // feed dog 1
  TIMERG1.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  TIMERG1.wdt_feed=1;                       // feed dog
  TIMERG1.wdt_wprotect=0;                   // write protect
  }
}

void enableSteppers(){
  digitalWrite((int)STEPPER_EN_PIN, LOW);
}

void disableSteppers(){
  digitalWrite((int)STEPPER_EN_PIN, HIGH);
}

bool steppersEnabled(){
  return (digitalRead((int)STEPPER_EN_PIN) == LOW);
}

float readADC(){
  uint32_t raw = adc1_get_raw(PSU_TEST_ADC_CHANNEL);
  uint32_t voltageMillivolts = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
  float voltage = voltageMillivolts / 1000.0f; // volts
  return voltage;
}

bool checkPSU(){
  // 20V power supply connected through 1:15 potential divider to PSU_TEST_PIN
  // Voltage is calculated using nominal 3.3V reference and resistor values
  // Using ESP32 calibrated readADC() function, not uncalibrated arduino analogRead()
  psuVoltage = readADC() * 16.0f;
  if (psuOn){
    psuOn = (psuVoltage >= PSU_VOLTAGE_LOWER);
  } else{
    psuOn = (psuVoltage >= PSU_VOLTAGE_UPPER);
  }

  return psuOn;
}

void failure(const char *msg){
  failed = true;
  disableSteppers();
  ErrorMessage errorMessage;
  strlcpy(errorMessage.msg, msg, sizeof(errorMessage.msg));
  errorMessage.state = FAILURE;
  xQueueSend(errorQueue, &errorMessage, 0);
}

void warning(const char *msg){
  ErrorMessage errorMessage;
  strlcpy(errorMessage.msg, msg, sizeof(errorMessage.msg));
  errorMessage.state = WARNING;
  xQueueSend(errorQueue, &errorMessage, 0);
}


void setupADC()
{
  // Configure ADC width (ESP32 supports 9–12 bits)
  adc1_config_width(ADC_WIDTH_BIT_12);

  // Configure attenuation for ADC1 channel 0 (GPIO36)
  adc1_config_channel_atten(PSU_TEST_ADC_CHANNEL, ADC_ATTEN_DB_11);

  // Characterise ADC using eFuse calibration if available
  esp_adc_cal_characterize(
      ADC_UNIT_1,
      ADC_ATTEN_DB_11,
      ADC_WIDTH_BIT_12,
      1100,                // default Vref if eFuse not present
      &adc_chars
  );
}

// Stepper driver class:
class StepperDriver{
public:
  // public variables:
  HomeState homeState = HOME_IDLE; // flag if position is homed (becomes HOME_COMPLETE)
  float travel; // in units
  float maxPos; // in units
  float minPos; // in units
  float maxSpeed;  // in units/s
  float maxAcceleration; // in units/s^2
  float defaultSpeed; // in units/s
  float defaultAcceleration;  // in units/s^2
  AccelStepper stepper{AccelStepper::DRIVER, 0, 0};
  MotorID motorID;
  StepperHardConfig hardConfig;
  StepperSoftConfig softConfig;
  volatile CmdType currentCmd = NONE;
  
  // Constructor:
  StepperDriver(StepperHardConfig hc,
                StepperSoftConfig sc,
                HardwareSerial &serialPort)
    : driver(&serialPort, hc.Rsense, hc.uartAddr),
      hardConfig(hc),
      softConfig(sc),
      motorID(hc.motorID),
      stepPin(hc.stepPin),
      dirPin(hc.dirPin),
      diagPin(hc.diagPin),
      swPin(hc.swPin),
      microsteps(sc.microsteps),
      stepsPerUnitPos(sc.stepsPerUnitPos),
      travel(sc.travel),
      maxSpeed(sc.maxSpeed),
      maxAcceleration(sc.maxAccel),
      defaultSpeed(sc.defaultSpeed),
      defaultAcceleration(sc.defaultAcceleration){}

  void init(){
    pinMode((int)stepPin, OUTPUT);
    pinMode((int)dirPin, OUTPUT);
    pinMode((int)diagPin, INPUT);
    pinMode((int)swPin, INPUT); // Limit switch is normally closed to GND (failsafe if disconnected), pulled up to 3.3V.

    // calculate parameters:
    maxSpeedMicrosteps = maxSpeed*stepsPerUnitPos*microsteps;
    maxAccelerationMicrosteps = maxAcceleration*stepsPerUnitPos*microsteps;
    
    // Setup stepper driver:
    driver.begin();
    driver.toff(5);
    driver.microsteps(microsteps);

    // AccelStepper object, note all units are in microsteps.
    stepper = AccelStepper(stepper.DRIVER, (int)stepPin, (int)dirPin);
    stepper.setMaxSpeed(maxSpeedMicrosteps);
    stepper.setAcceleration(maxAccelerationMicrosteps);
    stepper.setMinPulseWidth(20);

    // get home position from non-volatile memory:
    getHome();
    calculateLimits();

    Serial.printf("Motor %s initialised.\n", MOTOR_NAMES[motorID]);
  }

  // Public move function:
  bool moveTo(float position, float speed, float acceleration, char *response){
    int positionMicrosteps = position * stepsPerUnitPos * microsteps;
    int speedMicrosteps = speed * stepsPerUnitPos * microsteps;
    int accelMicrosteps = acceleration * stepsPerUnitPos * microsteps;
    if (positionMicrosteps < minPosMicrosteps || positionMicrosteps > maxPosMicrosteps){
      //warning("Position out of range, move not possible.");
      strlcpy(response, "WARNING! Position out of range, move not possible.", cmdParser::MAX_RESPONSE_SIZE);
    } else if (speedMicrosteps > maxSpeedMicrosteps){
      //warning("Speed out of range, move not possible.");
      strlcpy(response, "WARNING! Speed out of range, move not possible.", cmdParser::MAX_RESPONSE_SIZE);
    } else if (accelMicrosteps > maxAccelerationMicrosteps){
      //warning("Acceleration out of range, move not possible.");
      strlcpy(response, "WARNING! Acceleration out of range, move not possible.", cmdParser::MAX_RESPONSE_SIZE);
    } else{
      MoveCommand cmd;
      cmd.motorID=motorID;
      cmd.targetPos=positionMicrosteps;
      cmd.speed=speedMicrosteps;
      cmd.acceleration=accelMicrosteps;
      BaseType_t result = xQueueSend(motionQueue, &cmd, 0);
      if (result==errQUEUE_FULL){
        //warning("Motion queue full, move not sent.");
        strlcpy(response, "WARNING! Motion queue full, move not sent.", cmdParser::MAX_RESPONSE_SIZE);
        return false;
      } 
      return true;
    }
    //Serial.printf("pos: %.3f, speed: %.3f, accel: %.3f\n", position, speed, acceleration);
    return false;
  }

  // Public home function:
  bool homeAxis(char *response){
    MoveCommand cmd;
    cmd.motorID=motorID;
    cmd.home=true;
    BaseType_t result = xQueueSend(motionQueue, &cmd, 0);
    if (result==errQUEUE_FULL){
      strlcpy(response, "WARNING! Motion queue full, move not sent.", cmdParser::MAX_RESPONSE_SIZE);
      return false;
    }
    return true;
  }

  // Position readout:
  float getPosition() {
    return stepper.currentPosition();
  }

  void setMotionCmd(MoveCommand cmd){
    currentCmd = MOVE;
    stepper.setMaxSpeed(cmd.speed);
    stepper.setAcceleration(cmd.acceleration);
    stepper.moveTo(cmd.targetPos);
  }

  void run(){
    if (!limitTriggered() || homeState != HOME_COMPLETE){
      stepper.run();
    } else{
      failure("Limit switch triggered at an unexpected point. Reboot required.");
    }
    if (currentCmd == MOVE && stepper.distanceToGo() == 0){
      currentCmd = NONE;
      ReturnMessage returnMessage;
      returnMessage.motorID = motorID;
      returnMessage.pos = stepper.currentPosition();
      xQueueSend(returnQueue, &returnMessage, 0);
    }
  }

  void processHoming(){
    switch (homeState){
      case HOME_IDLE:
        return;

      case HOME_START:
        currentCmd = HOME;
        homeState = HOME_SEARCH;
        stepper.setMaxSpeed(softConfig.homeSearchSpeed * stepsPerUnitPos * microsteps);
        stepper.setAcceleration(softConfig.homeAccel * stepsPerUnitPos * microsteps);
        stepper.move(-softConfig.homeMaxDist * stepsPerUnitPos * microsteps);
        break;

      case HOME_SEARCH:
        if (limitTriggered()){
          stepper.stop();
          homeState = HOME_BACKOFF;
          stepper.move(softConfig.homeBackoff * stepsPerUnitPos * microsteps);
        } else if (stepper.distanceToGo() == 0){
          homeState = HOME_IDLE;
          failure("Homing failed, limit switch not triggered on search. Reboot required.");
        }
        break;

      case HOME_BACKOFF:
        if (stepper.distanceToGo() == 0){
          if (! limitTriggered()){
            homeState = HOME_LATCH;
            waitingToStop = false;
            stepper.setMaxSpeed(softConfig.homeLatchSpeed * stepsPerUnitPos * microsteps);
            stepper.move(-1.1 * softConfig.homeBackoff * stepsPerUnitPos * microsteps);
          } else{
            homeState = HOME_IDLE;
            failure("Homing failed, limit switch not released on backoff. Reboot required.");
          }
        }
        break;

      case HOME_LATCH:
        if (!waitingToStop){
          if (limitTriggered()){
            stepper.stop();
            home_pos_temp = stepper.currentPosition();
            waitingToStop = true;
          } else if (stepper.distanceToGo() == 0){
            homeState = HOME_IDLE;
            failure("Homing failed, limit switch not triggered on latch. Reboot required.");
          }
        } else if (waitingToStop && stepper.distanceToGo() == 0){
          homeState = HOME_OFFSET;
          stepper.setCurrentPosition(stepper.currentPosition() - home_pos_temp);
          stepper.setMaxSpeed(softConfig.homeSearchSpeed * stepsPerUnitPos * microsteps);
          stepper.moveTo(homeOffset);
          calculateLimits();
        }
      
      case HOME_OFFSET:
        if (stepper.distanceToGo() == 0){
          if (!limitTriggered()){
            homeState = HOME_COMPLETE;
            currentCmd = NONE;
            stepper.setCurrentPosition(0);
            ReturnMessage returnMessage;
            returnMessage.motorID = motorID;
            returnMessage.home = true;
            xQueueSend(returnQueue, &returnMessage, 0);
          } else{
            homeState = HOME_IDLE;
            failure("Homing failed, offset too close to limit switch. Reboot required.");
          }
        }

      case HOME_COMPLETE:
        return;
    }
  }

  void calculateLimits(){
    minPosMicrosteps = softConfig.homeBackoff*stepsPerUnitPos*microsteps - homeOffset;
    maxPosMicrosteps = minPosMicrosteps + travel*stepsPerUnitPos*microsteps;
    minPos = (float)minPosMicrosteps / ((float)stepsPerUnitPos * microsteps);
    maxPos = (float)maxPosMicrosteps / ((float)stepsPerUnitPos * microsteps);
  }

  bool setHome(char *response){
    // set current position to the home offset, store in EEPROM
    if (homeState != HOME_COMPLETE){
      strlcpy(response, "WARNING! Home offset cannot be set, must home axis first.", cmdParser::MAX_RESPONSE_SIZE);
    } else if (stepper.distanceToGo() > 0){
      strlcpy(response, "WARNING! Home offset cannot be set while in motion.", cmdParser::MAX_RESPONSE_SIZE);
    }
    else{
      long currentPos = stepper.currentPosition();  // this is the difference from the current home position
      homeOffset += currentPos;
      preferences.begin("home", false);
      preferences.putLong(MOTOR_NAMES[motorID], homeOffset);
      preferences.end();
      stepper.setCurrentPosition(0);
      calculateLimits();
      return true;
    }
    return false;
  }

  bool resetHome(char *response){
    // reset home offset to default value, store in EEPROM.
    if (stepper.distanceToGo() != 0){
      strlcpy(response, "WARNING! Home offset cannot be reset while in motion.", cmdParser::MAX_RESPONSE_SIZE);
    } else if (homeState != HOME_IDLE && homeState != HOME_COMPLETE){
      strlcpy(response, "WARNING! Home offset cannot be reset while homing.", cmdParser::MAX_RESPONSE_SIZE);
    } else{
      int deltaHomeOffset = homeOffset;
      homeOffset = softConfig.defaultHomeOffset*stepsPerUnitPos*microsteps;
      deltaHomeOffset = homeOffset - deltaHomeOffset; // new - old, correct sign
      preferences.begin("home", false);
      preferences.putLong(MOTOR_NAMES[motorID], homeOffset);
      preferences.end();
      if (homeState == HOME_COMPLETE){
        long currentPos = stepper.currentPosition();  // this is the difference from the old home position
        stepper.setCurrentPosition(currentPos - deltaHomeOffset);
        calculateLimits();
      }
      return true;
    }
    return false;
  }

  bool checkAvailable(){
    if (currentCmd == NONE){
      return true;
    } else{
      return false;
    }
  }

  void printData(){
    char *unit = MOTOR_UNITS[motorID];
    Serial.printf("\n\n\tStepper Driver %s\n\n", MOTOR_NAMES[motorID]);
    Serial.printf("\t\t%-24s %8.2f %s\n", "Minimum Position:", minPos, unit);
    Serial.printf("\t\t%-24s %8.2f %s\n", "Maximum Position:", maxPos, unit);
    Serial.printf("\t\t%-24s %8.2f %s/s\n", "Default Speed:", defaultSpeed, unit);
    Serial.printf("\t\t%-24s %8.2f %s/s/s\n", "Default Acceleration:", defaultAcceleration, unit);
    Serial.printf("\t\t%-24s %8.2f %s/s\n", "Maximum Speed:", maxSpeed, unit);
    Serial.printf("\t\t%-24s %8.2f %s/s/s\n", "Maximum Acceleration:", maxAcceleration, unit);
    Serial.printf("\t\t%-24s %8.2f %s\n", "Home Offset:", (float)homeOffset/(stepsPerUnitPos*microsteps), unit);
    Serial.printf("\t\t%-24s %8.2f %s\n", "Default Home Offset:", softConfig.defaultHomeOffset, unit);
    }


private:
  gpio_num_t stepPin, dirPin, diagPin, swPin;
  uint8_t uartAddr;
  uint16_t microsteps;  // nth fraction of a step
  uint16_t stepsPerUnitPos; // unit depends on motor (e.g. degrees or mm)
  int maxPosMicrosteps;
  int minPosMicrosteps;
  float maxSpeedMicrosteps; 
  float maxAccelerationMicrosteps;
  bool waitingToStop = false;
  long homeOffset; // microsteps
  long home_pos_temp = 0; // microsteps

  TMC2209Stepper driver;

  void getHome(){
    long defaultHomeOffset = softConfig.defaultHomeOffset*stepsPerUnitPos*microsteps;
    // returns value if it exists, otherwise returns default.
    preferences.begin("home", true);
    homeOffset = preferences.getLong(MOTOR_NAMES[motorID], defaultHomeOffset);
    // want to know if home position is not found:
    if (!preferences.isKey(MOTOR_NAMES[motorID])){
      Serial.println("WARNING! No home position saved, set to default.");
    }
    preferences.end();
  }

  bool limitTriggered(){
    return digitalRead((int)swPin) == HIGH;
  }

};


// setup stepper drivers:
StepperDriver stepperDriverRot(rotHardConfig, rotSoftConfig, stepperSerial);
StepperDriver stepperDriverLin1(lin1HardConfig, linSoftConfig, stepperSerial);
StepperDriver stepperDriverLin2(lin2HardConfig, linSoftConfig, stepperSerial);
 

int argToSteppers(const char arg[], StepperDriver *steppers[], int maxCount, MotorID *overallMotorID){
  // note maxCount must be 1-3;
  // returns 0 if argument invalid.
  int count = 0;
  if (strcmp(arg, MOTOR_ARGS[MOTOR_ROT]) == 0){
    steppers[count++] = &stepperDriverRot;
    *overallMotorID = MOTOR_ROT;
  } else if (strcmp(arg, MOTOR_ARGS[MOTOR_LIN1]) == 0){
    steppers[count++] = &stepperDriverLin1;
    *overallMotorID = MOTOR_LIN1;
  } else if (strcmp(arg, MOTOR_ARGS[MOTOR_LIN2]) == 0){
    steppers[count++] = &stepperDriverLin2;
    *overallMotorID = MOTOR_LIN2;
  } else if (strcmp(arg, MOTOR_ARGS[MOTOR_LIN]) == 0 && maxCount >= 2){
    steppers[count++] = &stepperDriverLin1;
    steppers[count++] = &stepperDriverLin2;
    *overallMotorID = MOTOR_LIN;
  } else if (strcmp(arg, MOTOR_ARGS[MOTOR_ALL]) == 0 && maxCount == 3){
    steppers[count++] = &stepperDriverRot;
    steppers[count++] = &stepperDriverLin1;
    steppers[count++] = &stepperDriverLin2;
    *overallMotorID = MOTOR_ALL;
  }
  return count;
}


void moveCmd(cmdParser::Argument *args, char *response, size_t numArgs){
  if (failed){
    strlcpy(response, failureMsg, cmdParser::MAX_RESPONSE_SIZE);
    return;
  }
  float pos = args[1].asDouble;
  float speed;
  float accel;
  StepperDriver *steppers[2] = {};
  MotorID motorID;  // overall ID for storing command
  
  int numSteppers = argToSteppers(args[0].asString, steppers, 2, &motorID);
  if (numSteppers > 0){
    // if moving both linear rails, check they are in the same position first.
    if (numSteppers == 2 && stepperDriverLin1.getPosition() != stepperDriverLin2.getPosition()){
      strlcpy(response, "WARNING! Linear axis motors are not at the same position, must home first.", cmdParser::MAX_RESPONSE_SIZE);
      return;
    }
    if (numArgs > 2){
      speed = args[2].asDouble;
    } else{
      speed = steppers[0]->defaultSpeed;
    }
    if (numArgs > 3){
      accel = args[3].asDouble;
    } else{
      accel = steppers[0]->defaultAcceleration;
    }
    for (int i=0; i<numSteppers; i++){
      // check position, speed and acceleration are within limits, and axis is homed:
      if (steppers[i]->homeState != HOME_COMPLETE){
        strlcpy(response, "WARNING! Axis not homed, must home before moving.", cmdParser::MAX_RESPONSE_SIZE);
        return; 
      } else if(pos < steppers[i]->minPos || pos > steppers[i]->maxPos){
        strlcpy(response, "WARNING! Position out of range", cmdParser::MAX_RESPONSE_SIZE);
        return;
      } else if (speed < 0 || speed > steppers[i]->maxSpeed){
        strlcpy(response, "WARNING! Speed out of range", cmdParser::MAX_RESPONSE_SIZE);
        return;
      } else if (accel < 0 || accel > steppers[i]->maxAcceleration){
        strlcpy(response, "WARNING! Acceleration out of range", cmdParser::MAX_RESPONSE_SIZE);
        return;
      } else if (!steppers[i]->checkAvailable()){
        strlcpy(response, "WARNING! Previous command still processing.", cmdParser::MAX_RESPONSE_SIZE);
        return;
      }
    }
    MoveCommand moveCommand;   // for storing in the currentCommands array;
    size_t numMotors = 0; 
    MotorID motorIDs[3];
    moveCommand.motorID = motorID;
    moveCommand.targetPos = pos;
    for (int i=0; i<numSteppers; i++){
      // send move command:
      if (steppers[i]->moveTo(pos,speed,accel,response)){
        motorIDs[numMotors] = steppers[i]->motorID;
        numMotors ++;
      } else{
        // don't continue if any didn't work, error is in *response.
        pushCommand(moveCommand, numMotors, motorIDs);
        return;
      }
    }
    pushCommand(moveCommand, numMotors, motorIDs);
    strlcpy(response, "Motor(s) moving...", cmdParser::MAX_RESPONSE_SIZE);
  } else{
    strlcpy(response, "WARNING! Invalid motor argument", cmdParser::MAX_RESPONSE_SIZE);
    return;
  }

}


void homeCmd(cmdParser::Argument *args, char *response, size_t numArgs){
  if (failed){
    strlcpy(response, failureMsg, cmdParser::MAX_RESPONSE_SIZE);
    return;
  }
  StepperDriver *steppers[3] = {};
  MotorID motorID;  // overall ID for storing command

  int numSteppers = argToSteppers(args[0].asString, steppers, 3, &motorID);
  if (numSteppers > 0){
    // check all are available first:
    for (int i=0; i<numSteppers; i++){
      if (!steppers[i]->checkAvailable()){
        strlcpy(response, "WARNING! Previous command still processing.", cmdParser::MAX_RESPONSE_SIZE);
        return;
      }
    }
    MoveCommand moveCommand;   // for storing in the currentCommands array;
    size_t numMotors = 0; 
    MotorID motorIDs[3];
    moveCommand.motorID = motorID;
    moveCommand.home = true;
    for (int i=0; i<numSteppers; i++){
      // send move command:
      if (steppers[i]->homeAxis(response)){
        motorIDs[numMotors] = steppers[i]->motorID;
        numMotors ++;
      } else{
        // don't continue if any didn't work, error is in *response.
        pushCommand(moveCommand, numMotors, motorIDs);
        return;
      }
    }
    pushCommand(moveCommand, numMotors, motorIDs);
    snprintf(response, cmdParser::MAX_RESPONSE_SIZE, "Homing axis %s...", args[0].asString);

  } else{
    strlcpy(response, "WARNING! Invalid motor argument", cmdParser::MAX_RESPONSE_SIZE);
    return;
  }

  // want to send a serial message when the command is complete, do this somewhere else.
}

void setHomeCmd(cmdParser::Argument *args, char *response, size_t numArgs){
  if (failed){
    strlcpy(response, failureMsg, cmdParser::MAX_RESPONSE_SIZE);
    return;
  }
  StepperDriver *steppers[3] = {};
  MotorID motorID;  // not used here
  int numSteppers = argToSteppers(args[0].asString, steppers, 3, &motorID);
  if (numSteppers > 0){
    for (int i=0; i<numSteppers; i++){
      if (steppers[i]->homeState == HOME_COMPLETE){
        if (steppers[i]->setHome(response)){
          strlcpy(response, "COMPLETED: Current position stored as home in non-volatile memory.", cmdParser::MAX_RESPONSE_SIZE);
        } else{
          // don't continue if any didn't work, error is in *response.
          return;
        }
      } else{
        strlcpy(response, "WARNING! Axis not homed, cannot store position.", cmdParser::MAX_RESPONSE_SIZE);
        return;
      }
    }
  } else{
    strlcpy(response, "WARNING! Invalid motor argument", cmdParser::MAX_RESPONSE_SIZE);
    return;
  }
}

void resetHomeCmd(cmdParser::Argument *args, char *response, size_t numArgs){
  if (failed){
    strlcpy(response, failureMsg, cmdParser::MAX_RESPONSE_SIZE);
    return;
  }
  StepperDriver *steppers[3] = {};
  MotorID motorID;  // not used here
  int numSteppers = argToSteppers(args[0].asString, steppers, 3, &motorID);
  if (numSteppers > 0){
    for (int i=0; i<numSteppers; i++){
      if (steppers[i]->resetHome(response)){
        strlcpy(response, "COMPLETED: Home position reset to default.", cmdParser::MAX_RESPONSE_SIZE);
      } else{
        // don't continue if any didn't work, error is in *response.
        return;
      }
    }
  } else{
    strlcpy(response, "WARNING! Invalid motor argument", cmdParser::MAX_RESPONSE_SIZE);
    return;
  }
}

void rebootCmd(cmdParser::Argument *args, char *response, size_t numArgs){
  reboot();
}

void getVoltageCmd(cmdParser::Argument *args, char *response, size_t numArgs){
  float voltage = psuVoltage;
  snprintf(response, cmdParser::MAX_RESPONSE_SIZE, "PSU voltage: %.3f V.", voltage);
}

void helpCmd(cmdParser::Argument *args, char *response, size_t numArgs){
  Serial.println("\nAutomatic Wake Bar System (AWBS)\n");
  Serial.println("Designed by Daniel Harrop, 2026, for the Whittle Laboratory mPACT rig.\n");
  Serial.println("ESP32 Microcontroller code can be found at: https://github.com/dh696/mPACT_AWBS/\n\n");

  Serial.println("Serial Commands:");

  Serial.println("\n\tmove <axis> <position> [<speed>] [<acceleration>]");
  Serial.println("\t\tMove selected <axis> to absolute coordinate <position> from home (zero).");
  Serial.println("\t\tOptionally specify <speed> and <acceleration>, otherwise these are default.");
  Serial.println("\t\t<axis> must be one of [\"rot\", \"lin\", \"lin1\", \"lin2\"].");
  Serial.println("\t\tUnits of <position> are degrees for rotation axis, and millimeters for linear axis.");
  Serial.println("\t\tUnits of <speed> and <acceleration> are the same, per second (per second).");

  Serial.println("\n\thome <axis>");
  Serial.println("\t\tLocate and move to coordinate zero for selected <axis>.");
  Serial.println("\t\t<axis> must be one of [\"all\", \"rot\", \"lin\", \"lin1\", \"lin2\"]");
  Serial.println("\t\tOnce homed, limit switches must not be pressed.");

  Serial.println("\n\tsetHome <axis>");
  Serial.println("\t\tChange the home offset to the current position for selected <axis>.");
  Serial.println("\t\t<axis> must be one of [\"all\", \"rot\", \"lin\", \"lin1\", \"lin2\"]");
  Serial.println("\t\tThe current position becomes coordinate zero.");
  Serial.println("\t\tHome offset is stored in non-volatile memory, deleting the previous value permanantly.");

  Serial.println("\n\tresetHome <axis>");
  Serial.println("\t\tChange the home offset to the default position for selected <axis>.");
  Serial.println("\t\t<axis> must be one of [\"all\", \"rot\", \"lin\", \"lin1\", \"lin2\"]");
  Serial.println("\t\tThe default home offset is hard-coded in case \"setHome\" is run in error.");
  Serial.println("\t\tHome offset is stored in non-volatile memory, deleting the previous value permanantly.");

  Serial.println("\n\tpsu");
  Serial.println("\t\tDisplay the current power supply voltage");

  Serial.println("\n\tdata");
  Serial.println("\t\tDisplay current settings, default values and limits.");

  Serial.println("\n\thelp");
  Serial.println("\t\tDisplay this page.");

  Serial.println("\n\treboot");
  Serial.println("\t\tRestart the ESP32 microcontroller.");
  Serial.println("\t\tThis must be run if an \"ERROR!\" is returned at any point.");
  Serial.println("\t\tAlternatively, disconnect and reconnect the USB cable or motor power supply.\n");
}

void dataCmd(cmdParser::Argument *args, char *response, size_t numArgs){
  Serial.println("\nAutomatic Wake Bar System (AWBS)\n\n");
  Serial.println("Current Settings, default Values and limits: ");
  stepperDriverRot.printData();
  stepperDriverLin1.printData();
  stepperDriverLin2.printData();

}



void reboot(){
  disableSteppers();
  Serial.println("Rebooting...");
  ESP.restart();
  // this doesn't fully restart the device (pins remain in same state), may want to connect GPIO 33 to the EN pin to hardware reset.
}



void pushCommand(MoveCommand moveCommand, size_t numMotors, MotorID motorIDs[3]){
  for (int i=0; i<3; i++){
    if (currentCommands[i].remainingReturns == 0){
      currentCommands[i].moveCommand = moveCommand;
      currentCommands[i].remainingReturns = numMotors;
      for (int j=0; j<numMotors; j++){
        currentCommands[i].waitingForMsgs[j].home = moveCommand.home;
        currentCommands[i].waitingForMsgs[j].pos = moveCommand.targetPos;
        currentCommands[i].waitingForMsgs[j].motorID = motorIDs[j];
      }
      for (int j=numMotors; j<3; j++){
        // reset remaining messages
        currentCommands[i].waitingForMsgs[j].motorID = MOTOR_NONE;
      }
      return;
    }
  }
}

void checkReturn(ReturnMessage returnMessage){
  for (int i=0; i<3; i++){
    if (currentCommands[i].remainingReturns > 0){
      for (int j=0; j<3; j++){
        if (currentCommands[i].waitingForMsgs[j].motorID == returnMessage.motorID){
          currentCommands[i].remainingReturns --;
          if (currentCommands[i].remainingReturns == 0){
            // full command completed:
            Serial.printf("COMPLETED: %s %s %.2f\n", currentCommands[i].moveCommand.home ? "home" : "move", MOTOR_ARGS[currentCommands[i].moveCommand.motorID], currentCommands[i].moveCommand.targetPos);
            return;
          }
        }
      }
    }
  }
}



void motionTask(void *pvParameters){
  MoveCommand cmd;
  ErrorMessage errorMessage;
  for (;;){
    // Check PSU:
    checkPSU();
    if (!psuOn && steppersEnabled()){
      // PSU has just turned off, throw error:
      failure("Power supply is off, motors disabled.");
      waitingForPower = true;
    } else if (psuOn && waitingForPower){
      // PSU has just turned on, reboot:
      reboot();
    }
    // Check if a command is received:
    while (xQueueReceive(motionQueue, &cmd, 0) == pdTRUE){
      StepperDriver *motor = nullptr;
      switch (cmd.motorID){
        case MOTOR_ROT: motor = &stepperDriverRot; break;
        case MOTOR_LIN1: motor = &stepperDriverLin1; break;
        case MOTOR_LIN2: motor = &stepperDriverLin2; break;
      }
      if (motor){
        if (!failed){
          if (cmd.home){
            motor->homeState = HOME_START;
          } else{
            motor->setMotionCmd(cmd);
          }
        }
      }
    }
    stepperDriverRot.processHoming();
    stepperDriverLin1.processHoming();
    stepperDriverLin2.processHoming();
    if (!failed){
      stepperDriverRot.run();
      stepperDriverLin1.run();
      stepperDriverLin2.run();
    }
    feedTheDog(0);  // to prevent watchdog resetting ESP32
  }
}




void setup()
{  
  // Serial port to PC:
  Serial.begin(115200);
  // delay for stability:
  delay(3000);
  Serial.flush();
  Serial.println("\nAutomatic Wake Bar System\nStarting...");

  // UART Port 2 for stepper drivers:
  stepperSerial.begin(SERIAL_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(1000);

  // Enable pin for all stepper drivers, 0=enabled, 1=disabled
  pinMode((int)STEPPER_EN_PIN, OUTPUT);
  disableSteppers();

  // Power Supply test pin, 20V motor supply connected through 15:1 potential divider.
  pinMode((int)PSU_TEST_PIN, INPUT);
  setupADC();

  // Initialise stepper drivers:
  stepperDriverRot.init();
  delay(100);
  stepperDriverLin1.init();
  delay(100);
  stepperDriverLin2.init();

  // serial command parser:
  parser.registerCommand("move", "sddd", &moveCmd);
  parser.registerCommand("move", "sdd", &moveCmd);
  parser.registerCommand("move", "sd", &moveCmd);
  parser.registerCommand("home", "s", &homeCmd);
  parser.registerCommand("setHome", "s", &setHomeCmd);
  parser.registerCommand("resetHome", "s", &resetHomeCmd);
  parser.registerCommand("reboot", "", &rebootCmd);
  parser.registerCommand("psu", "", &getVoltageCmd);
  parser.registerCommand("help", "", &helpCmd);
  parser.registerCommand("data", "", &dataCmd);

  // Queue for stepper commands
  motionQueue = xQueueCreate(10,sizeof(MoveCommand));
  returnQueue = xQueueCreate(10,sizeof(ReturnMessage));
  errorQueue = xQueueCreate(10,sizeof(ErrorMessage));

  // Start motion task on second core to send the stepper pulses:
  xTaskCreatePinnedToCore(
      motionTask,       // task function
      "motion_task",    // name
      4096,             // stack size
      NULL,             // parameters
      2,                // priority
      NULL,             // task handle
      0                 // core number: run on other core (main loop on core 1)
    );
  
  enableSteppers();

  delay(100);
  if (!failed){
    Serial.println("STARTED");
  }
}

void loop()
{
  ReturnMessage returnMessage;
  ErrorMessage errorMessage;
  char response[cmdParser::MAX_RESPONSE_SIZE];
  char buffer[80];
  if (Serial.available()){
    incomingChar = Serial.read();
    if (incomingChar != '\n'){
      line[charCount] = incomingChar;
      charCount += 1;
      if (charCount >= sizeof(line)-1){
        // limit line length
        charCount = 0;
        line[0] = '\0';
      }
    }
    else{ // End of message, now process command:
      line[charCount] = '\0';
      if (parser.processCommand(line, response)){
        Serial.println(response);
      }    
      else{
        Serial.println("Invalid command received:");
        Serial.println(line);
        Serial.println(response);
      }
      charCount = 0;
      line[0] = '\0';
    }
  } 
  if (xQueueReceive(returnQueue, &returnMessage, 0) == pdTRUE){
    checkReturn(returnMessage);
  }
  if (xQueueReceive(errorQueue, &errorMessage, 0) == pdTRUE){
    if (errorMessage.state == WARNING){
      strlcpy(buffer, "WARNING! ", sizeof(buffer));
    } else if (errorMessage.state == FAILURE){
      strlcpy(buffer, "ERROR! ", sizeof(buffer));
    }
    // print error to serial:
    strlcat(buffer, errorMessage.msg, sizeof(buffer));
    Serial.println(buffer);
    // Store for sending error if further commands are sent.
    if (errorMessage.state = FAILURE){
      strlcpy(failureMsg, buffer, sizeof(failureMsg));
    }
    buffer[0] = 0;
  }
}

