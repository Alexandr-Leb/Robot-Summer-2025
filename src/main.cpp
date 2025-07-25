// --- Header Files --- //
#include <Arduino.h>
#include "driver/adc.h"
#include "driver/ledc.h"
#include <math.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

// --- Pins --- //
// Pins - Motors
#define LEFT_MOTOR_FORWARDS_PIN 25
#define LEFT_MOTOR_REVERSE_PIN 26
#define RIGHT_MOTOR_FORWARDS_PIN 33
#define RIGHT_MOTOR_REVERSE_PIN 32
#define VERTICAL_MOTOR_FORWARDS_PIN 15
#define VERTICAL_MOTOR_REVERSE_PIN 2

// Pins - Servos
#define PIN_BASE_ROT     5
#define PIN_SHOULDER     4
#define PIN_ELBOW       14
#define PIN_WRIST       12
#define PIN_CLAW        13

// Pins - Sensors
#define LEFT_REFLECTANCE_PIN ADC1_CHANNEL_2 // 38
#define RIGHT_REFLECTANCE_PIN ADC1_CHANNEL_1 // 37
#define MAGNETOMETER_PIN_1 21
#define MAGNETOMETER_PIN_2 22

// --- Channels --- //
// Channels - Drive Motors
#define LEFT_FORWARDS_CHANNEL 0
#define LEFT_REVERSE_CHANNEL 1
#define RIGHT_FORWARDS_CHANNEL 2
#define RIGHT_REVERSE_CHANNEL 3
#define VERTICAL_FORWARDS_CHANNEL 4
#define VERTICAL_REVERSE_CHANNEL 5

// Channels - Arm Servos
#define CH_BASE_ROT      6
#define CH_SHOULDER      7
#define CH_ELBOW         8
#define CH_WRIST         9
#define CH_CLAW         10

// --- Constants --- //
// Constants - Arm Servos
#define SERVO_PWM_FREQ_HZ   50          // 20 ms period
#define SERVO_PWM_RES_BITS  16          // 0‑65535
#define SERVO_PERIOD_US     (1000000UL / SERVO_PWM_FREQ_HZ)
#define SERVO_MAX_COUNT     ((1UL << SERVO_PWM_RES_BITS) - 1)

#define JOINT_BASE_ROT   0
#define JOINT_SHOULDER   1
#define JOINT_ELBOW      2
#define JOINT_WRIST      3
#define JOINT_CLAW       4
#define JOINT_COUNT      5

static const uint8_t JOINT_PIN[JOINT_COUNT] = {
    PIN_BASE_ROT, PIN_SHOULDER, PIN_ELBOW, PIN_WRIST, PIN_CLAW
};
static const uint8_t JOINT_CH[JOINT_COUNT]  = {
    CH_BASE_ROT,  CH_SHOULDER,  CH_ELBOW,  CH_WRIST,  CH_CLAW
};

#define MIN_US_BASE_ROT   550
#define MIN_US_SHOULDER   550
#define MIN_US_ELBOW      550
#define MIN_US_WRIST      550
#define MIN_US_CLAW       550

#define MAX_US_BASE_ROT  2450
#define MAX_US_SHOULDER  2450
#define MAX_US_ELBOW     2450
#define MAX_US_WRIST     2450
#define MAX_US_CLAW      2450

static const uint16_t JOINT_MIN_US[JOINT_COUNT] = {
    MIN_US_BASE_ROT, MIN_US_SHOULDER, MIN_US_ELBOW, MIN_US_WRIST, MIN_US_CLAW
};
static const uint16_t JOINT_MAX_US[JOINT_COUNT] = {
    MAX_US_BASE_ROT, MAX_US_SHOULDER, MAX_US_ELBOW, MAX_US_WRIST, MAX_US_CLAW
};

#define SPEED_IMMEDIATE_DEGPS   10000.0f  // “jump now”
#define SLEW_BASE_ROT           20.0f
#define SLEW_SHOULDER           40.0f
#define SLEW_ELBOW              40.0f
#define SLEW_WRIST              40.0f
#define SLEW_CLAW               SPEED_IMMEDIATE_DEGPS

static const float JOINT_SLEW_DEGPS[JOINT_COUNT] = {
    SLEW_BASE_ROT, SLEW_SHOULDER, SLEW_ELBOW, SLEW_WRIST, SLEW_CLAW
};

struct JointState {
  float    currentDeg;    // PWM'd position
  float    targetDeg;     // user target
  uint32_t lastUpdateMs;  // timestamp of last currentDeg change
};
static JointState joints[JOINT_COUNT]; //array for keeping track of joint states

static inline float clampDeg(float d) {
  return (d < 0.0f) ? 0.0f : (d > 180.0f ? 180.0f : d);
}

static inline uint16_t angleToUs(uint8_t joint, float deg) {
  deg = clampDeg(deg);
  uint16_t usMin = JOINT_MIN_US[joint];
  uint16_t usMax = JOINT_MAX_US[joint];
  return usMin + (uint16_t)((usMax - usMin) * (deg / 180.0f) + 0.5f);
}

static inline uint32_t usToDuty(uint16_t us) {
  uint32_t duty = ((uint64_t)us * SERVO_MAX_COUNT) / SERVO_PERIOD_US;
  return (duty > SERVO_MAX_COUNT) ? SERVO_MAX_COUNT : duty;
}

using Joint = uint8_t;

// Constants - Arm Lengths
#define BICEP_LENGTH 132.446
#define FOREARM_LENGTH 105.641

// Constants - Arm Offsets
#define SHOULDER_OFFSET_ANGLE 10
#define ELBOW_OFFSET_ANGLE 30 // From 37, decreasing brings elbow up
#define WRIST_OFFSET_ANGLE 70 // From 70, increasing brings wrist up

// Constants - Tape Following
#define REFLECTANCE_THRESHOLD_OFFSET 400
#define HYSTERESIS_MULTIPLIER 50

// Constants - Magnetometer
#define MAGNETOMETER_THRESHOLD 260

// Constants - Motors
#define MOTOR_PWM_FREQUENCY 120 // in Hz
#define MOTOR_PWM_NUM_BITS 12
#define MOTOR_SWITCH_TIME 5 // in us
#define MOTOR_STOP_CORRECTION 1.5

// --- Variables --- //
// Variables - State
enum MasterState {Initialize, Test, ClearDoorway, Pet1, ClimbRamp, Pet2, Pet3};
MasterState currentMasterState;

enum ProcedureState {TapeFollow, TapeFind, Stop, PetSearch, PetReach, PetGrab, PreState, PostState};
ProcedureState currentProcedureState;

// Variables - Time Control
unsigned long prevTimeRecord;

// Variables - Magnetometer
Adafruit_LIS3MDL lis;
const float threshold = 1.0;  // Minimum change to consider significant (uT)
double magnetometerMagnitude;
double magnetometerMagnitudeSum;
int magnetometerAverageCount;

// Variables - Reflectance Sensor
int leftReflectance;
int rightReflectance;

// Variables - Motors
int leftMotorPrev;
unsigned long leftTimeSwitch; // in us
int rightMotorPrev;
unsigned long rightTimeSwitch; // in us
int verticalMotorPrev;
unsigned long verticalTimeSwitch; // in us
int stopPower;

// Variables - PID
double k_p;
double k_i; // Not using
double k_d;
double pValue;
double iValue; // Not using
double dValue;
int error;
int prevError;
long prevTime; // in us
double k_e;
double eValue;

// Variables - Hysteresis
bool leftOnTape;
bool rightOnTape;
bool prevLeftOnTape;
bool prevRightOnTape;
int leftReflectanceThreshold;
int rightReflectanceThreshold;

// Variables - Reflectance Threshold Average Calculation
int leftReflectanceThresholdSum;
int rightReflectanceThresholdSum;
int reflectanceAverageLoopCounter;

// Run Time Constants
// Run Time Constants - ClearDoorway
#define DOORWAY_CLEAR_TIME 3950
#define ARM_DEPLOY_TIME 2000
#define DOORWAY_STOP_TIME 1000
#define DOORWAY_STOP_NUM_INCREMENTS 5
#define BAKSET_CLEAR_TIME 3000

// Run Time Constants - Pet1
#define PET1_ARM_DEPLOY_TIME 2000
#define PET1_SWEEP_PM_ANGLE 30
double pet1_magnetometerSweepArray[2 * PET1_SWEEP_PM_ANGLE + 1];
#define PET1_SWEEP_TIME 5000
int pet1_searchTimePerDegree = PET1_SWEEP_TIME / (2 * PET1_SWEEP_PM_ANGLE + 1);
int pet1_degreeIncrementCounter = 0;
int maxMagnetometerValue = 0;
int maxMagnetometerArmAngle;
float newShoulderTarget;
float newElbowTarget;
float newWristTarget;

// Run Time Constants - ClimbRamp
#define RAMP_FIND_TIME 5000

// --- Function Headers --- //
// Sensor Functions
void readMagnetometer();
void readReflectanceSensors(); // Reads and computes hysteresis variables
void initializeReflectanceSensors(int time); // time in ms

// Drivetrain Functions
void computePID();
void runPID(int power); // Power between 0 and 4095
void runHysteresis(int power);
void rightMotor_SetPower(int power); // Negative power for reverse, between -4095 and 4095
void leftMotor_SetPower(int power); // Negative power for reverse, between -4095 and 4095
void verticalMotor_SetPower(int power); // Negative power for reverse, between -4095 and 4095

// Servo FUnctions
static void writeServoRaw(Joint j, float deg);
void setJointTarget(Joint j, float deg);
void setAllTargets(float baseDeg, float shoulderDeg, float elbowDeg, float wristDeg, float clawDeg);
void updateServos();
void waitForServos();
double computeElbowAngle(double shoulderAngle, double height);
double computeWristAngle(double shoulderAngle, double elbowAngle, double height);

// --- Setup --- //
void setup() {
  // Variables - State
  currentMasterState = MasterState::ClearDoorway;
  currentProcedureState = ProcedureState::PreState;

  // Variables - Time Control
  prevTimeRecord = 0;

  // Variables - Magnetometer
  lis.begin_I2C();
  magnetometerMagnitude = 0.0;
  magnetometerMagnitudeSum = 0.0;
  magnetometerAverageCount = 0;

  // Variables - Reflectance Sensors
  leftReflectance = 0;
  rightReflectance = 0;

  // Variables - Motors
  leftMotorPrev = 0;
  leftTimeSwitch = 0;
  rightMotorPrev = 0;
  rightTimeSwitch = 0;
  verticalMotorPrev = 0;
  verticalTimeSwitch = 0;

  // Variables - PID
  k_p = 1.5;
  k_i = 0.0;
  k_d = 2.0;
  pValue = 0.0;
  iValue = 0.0;
  dValue = 0.0;
  error = 0;
  prevError = 0;
  prevTime = 0;
  k_e = 0.2;
  eValue = 0.0;

  // Variables - Hysteresis
  leftOnTape = true;
  rightOnTape = true;
  prevLeftOnTape = true;
  prevRightOnTape = true;

  // Variables - Reflectance Threshold Average Calculation
  leftReflectanceThresholdSum = 0;
  rightReflectanceThresholdSum = 0;
  reflectanceAverageLoopCounter = 0;

  // Variables - PRE CALIBRATION
  leftReflectanceThreshold = 2900;
  rightReflectanceThreshold = 2900;

  // Reflectance Sensor Analog Input Setup
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(LEFT_REFLECTANCE_PIN, ADC_ATTEN_DB_12);
  adc1_config_channel_atten(RIGHT_REFLECTANCE_PIN, ADC_ATTEN_DB_12);

  // Motor PWM Setup
  ledcSetup(LEFT_FORWARDS_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcSetup(LEFT_REVERSE_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcSetup(RIGHT_FORWARDS_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcSetup(RIGHT_REVERSE_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcSetup(VERTICAL_FORWARDS_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcSetup(VERTICAL_REVERSE_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcAttachPin(LEFT_MOTOR_FORWARDS_PIN, LEFT_FORWARDS_CHANNEL);
  ledcAttachPin(LEFT_MOTOR_REVERSE_PIN, LEFT_REVERSE_CHANNEL);
  ledcAttachPin(RIGHT_MOTOR_FORWARDS_PIN, RIGHT_FORWARDS_CHANNEL);
  ledcAttachPin(RIGHT_MOTOR_REVERSE_PIN, RIGHT_REVERSE_CHANNEL);
  ledcAttachPin(VERTICAL_MOTOR_FORWARDS_PIN, VERTICAL_FORWARDS_CHANNEL);
  ledcAttachPin(VERTICAL_MOTOR_REVERSE_PIN, VERTICAL_REVERSE_CHANNEL);

  // Arm Setup
  for (uint8_t j = 0; j < JOINT_COUNT; ++j) {
    ledcSetup(JOINT_CH[j], SERVO_PWM_FREQ_HZ, SERVO_PWM_RES_BITS);
    ledcAttachPin(JOINT_PIN[j], JOINT_CH[j]);

    // Start each joint at a *known* angle (90 ° = neutral)
    joints[j].currentDeg   = 90.0f;          // software believes we're at 90 °
    joints[j].targetDeg    = 90.0f;          // same here → no immediate motion
    joints[j].lastUpdateMs = millis();       // time‑stamp the “write”
    writeServoRaw(j, 90.0f);                 // send the 90 ° PWM pulse now

  }
  const float TOL_DEG = 0.05f;         // tolerance for knowing if a servo is still moving
  bool settling = true;
  while (settling) {
    updateServos();                    // one incremental step

    // Check if any joint is still moving
    settling = false;
    for (uint8_t j = 0; j < JOINT_COUNT; ++j) {
      if (fabsf(joints[j].currentDeg - joints[j].targetDeg) > TOL_DEG) {
        settling = true;
        break;
      }
    }
    delay(5);                         
  }
}

// --- Loop --- //
void loop() {
  switch(currentMasterState) {
    case MasterState::ClearDoorway:
    switch(currentProcedureState) {
      case ProcedureState::PreState:
      setAllTargets(90, 80, 5, 0, 90);
      updateServos();
      if (millis() - prevTimeRecord > ARM_DEPLOY_TIME) {
        currentProcedureState = ProcedureState::TapeFollow;
        prevTimeRecord = millis();
      }
      break;

      case ProcedureState::TapeFollow:
      runPID(1000);
      if (!leftOnTape && !rightOnTape) {
        currentProcedureState = ProcedureState::TapeFind;
      }
      if (millis() - prevTimeRecord > DOORWAY_CLEAR_TIME) {
        // currentProcedureState = ProcedureState::Stop;
        // stopPower = 800;
        // prevTimeRecord = millis();
        currentProcedureState = ProcedureState::PostState;
        leftMotor_SetPower(0);
        rightMotor_SetPower(0);
        prevTimeRecord = millis();
      }
      break;

      case ProcedureState::TapeFind:
      readReflectanceSensors();
      runHysteresis(1000);
      if (leftOnTape || rightOnTape) {
        computePID();
        currentProcedureState = ProcedureState::TapeFollow;
      }
      if (millis() - prevTimeRecord > DOORWAY_CLEAR_TIME) {
      //   currentProcedureState = ProcedureState::Stop;
      //   stopPower = 800;
      //   prevTimeRecord = millis();
        currentProcedureState = ProcedureState::PostState;
        leftMotor_SetPower(0);
        rightMotor_SetPower(0);
        prevTimeRecord = millis();
      }
      break;

      // case ProcedureState::Stop:
      // if (millis() - prevTimeRecord > DOORWAY_STOP_TIME / DOORWAY_STOP_NUM_INCREMENTS) {
      //   stopPower -= 200;
      //   readReflectanceSensors();
      //   computePID();
      //   if (stopPower > 0) {
      //     leftMotor_SetPower((int) ((stopPower + pValue + dValue - abs(eValue)) * MOTOR_STOP_CORRECTION));
      //     rightMotor_SetPower((int) ((stopPower - pValue - dValue - abs(eValue)) * MOTOR_STOP_CORRECTION));
      //   } else {
      //     leftMotor_SetPower(0);
      //     rightMotor_SetPower(0);
      //     currentProcedureState = ProcedureState::PostState;
      //   }
      //   prevTimeRecord = millis();
      // }
      // break;

      case ProcedureState::PostState:
      verticalMotor_SetPower(2500);
      if (millis() - prevTimeRecord > BAKSET_CLEAR_TIME) {
        verticalMotor_SetPower(0);
        currentMasterState = MasterState::Pet1;
        currentProcedureState = ProcedureState::PreState;
        setJointTarget(0, 30);
        updateServos();
        delay(1000);
        prevTimeRecord = millis();
      }
      break;
    }
    break;

    case MasterState::Pet1:
    switch(currentProcedureState) {
      case ProcedureState::PreState:
      setAllTargets(30, 120, 30, 40, 100);
      updateServos();
      if (millis() - prevTimeRecord > PET1_ARM_DEPLOY_TIME) {
        currentProcedureState = ProcedureState::PetSearch;
        prevTimeRecord = millis();
      }
      break;

      case ProcedureState::PetSearch:
      readMagnetometer();
      magnetometerMagnitudeSum += magnetometerMagnitude;
      magnetometerAverageCount++;
      if (millis() - prevTimeRecord > pet1_searchTimePerDegree) {
        pet1_magnetometerSweepArray[pet1_degreeIncrementCounter] = magnetometerMagnitudeSum / magnetometerAverageCount;
        magnetometerMagnitudeSum = 0.0;
        magnetometerAverageCount = 0;
        pet1_degreeIncrementCounter++;
        writeServoRaw(0, 30 + pet1_degreeIncrementCounter);
        if (pet1_degreeIncrementCounter == 2 * PET1_SWEEP_PM_ANGLE + 1) {
          currentProcedureState = ProcedureState::PetReach;
          for (int degreeIncrement = 0; degreeIncrement < 2 * PET1_SWEEP_PM_ANGLE + 1; degreeIncrement++) {
            if (pet1_magnetometerSweepArray[degreeIncrement] > maxMagnetometerValue) {
              maxMagnetometerValue = pet1_magnetometerSweepArray[degreeIncrement];
              maxMagnetometerArmAngle = 30 + degreeIncrement;
            }
          }
          setAllTargets(maxMagnetometerArmAngle, 120, 30, 40, 100);
          updateServos();
          waitForServos();
        }
        prevTimeRecord = millis();
      }
      break;

      case ProcedureState::PetReach:
      newShoulderTarget = joints[1].currentDeg + 1;
      newElbowTarget = computeElbowAngle(newShoulderTarget, 35);
      newWristTarget = computeWristAngle(newShoulderTarget, newElbowTarget, 35);
      setAllTargets(joints[0].currentDeg, newShoulderTarget, newElbowTarget, newWristTarget, joints[4].currentDeg);
      waitForServos();
      readMagnetometer();
      if (magnetometerMagnitude > MAGNETOMETER_THRESHOLD) {
        currentProcedureState = ProcedureState::PetGrab;
      }
      if (newShoulderTarget > 120) {
        currentProcedureState = ProcedureState::PetGrab;
      }
      break;

      case ProcedureState::PetGrab:
      setJointTarget(4, 5);
      waitForServos();
      delay(800);
      setAllTargets(175, 80, 5, 0, 5);
      waitForServos();
      delay(3000);
      currentProcedureState = ProcedureState::PostState;
      prevTimeRecord = millis();
      break;

      case ProcedureState::PostState:
      currentMasterState = MasterState::ClimbRamp;
      currentProcedureState = ProcedureState::PreState;
      break;
    }
    break;

    case MasterState::ClimbRamp:
    switch(currentProcedureState) {
      case PreState:
      readReflectanceSensors();
      leftMotor_SetPower(-800);
      rightMotor_SetPower(-800);
      if (leftOnTape || rightOnTape) {
        computePID();
        currentProcedureState = ProcedureState::TapeFollow;
      }
      break;

      case TapeFollow:
      runPID(1000);
      if (!leftOnTape && !rightOnTape) {
        currentProcedureState = ProcedureState::TapeFind;
      }
      if (millis() - prevTimeRecord > RAMP_FIND_TIME) {
        leftMotor_SetPower(0);
        rightMotor_SetPower(0);
        currentProcedureState = ProcedureState::PostState;
      }
      break;

      case TapeFind:
      readReflectanceSensors();
      runHysteresis(1000);
      if (leftOnTape || rightOnTape) {
        computePID();
        currentProcedureState = ProcedureState::TapeFollow;
      }
      if (millis() - prevTimeRecord > RAMP_FIND_TIME) {
        leftMotor_SetPower(0);
        rightMotor_SetPower(0);
        currentProcedureState = ProcedureState::PostState;
      }
      break;

      case PostState:
      setAllTargets(175, 120, 30, 40, 100);
      waitForServos();
      delay(3000);
      setJointTarget(4, 90);
      waitForServos();
      delay(1000000);
      break;
    }
    break;

    // case MasterState::Pet2:
    // break;

    // case MasterState::Pet3:
    // break;

    // case MasterState::Test:
    // switch(currentProcedureState) {
    //   case ProcedureState::TapeFollow:
    //   runPID(1000);
    //   if (!leftOnTape && !rightOnTape) {
    //     currentProcedureState = ProcedureState::TapeFind;
    //   }
    //   break;

    //   case ProcedureState::TapeFind:
    //   readReflectanceSensors();
    //   runHysteresis(1000);
    //   if (leftOnTape || rightOnTape) {
    //     computePID();
    //     currentProcedureState = ProcedureState::TapeFollow;
    //   }
    //   break;
    // }
    // setAllTargets(90, 60, 0, 0, 90);
    // updateServos();
    // break;

    case MasterState::Initialize:
    setAllTargets(0, 80, 0, 50, 90);
    updateServos();
    break;
  }
}

// --- Function Definitions --- //
void readMagnetometer() {
  sensors_event_t event;
  lis.getEvent(&event);
  float x = event.magnetic.x;
  float y = event.magnetic.y;
  float z = event.magnetic.z;
  magnetometerMagnitude = (double) sqrt(x * x + y * y + z * z);
}

void readReflectanceSensors() {
  leftReflectance = adc1_get_raw(LEFT_REFLECTANCE_PIN);
  rightReflectance = adc1_get_raw(RIGHT_REFLECTANCE_PIN);
  
  // Hysteresis Computation
  prevLeftOnTape = leftOnTape;
  prevRightOnTape = rightOnTape;
  leftOnTape = (leftReflectance > leftReflectanceThreshold);
  rightOnTape = (rightReflectance > rightReflectanceThreshold);
}

void initializeReflectanceSensors(int time) {
  int startTime = millis();
  while (millis() - startTime < time) {
    leftReflectanceThresholdSum += adc1_get_raw(LEFT_REFLECTANCE_PIN);
    rightReflectanceThresholdSum += adc1_get_raw(RIGHT_REFLECTANCE_PIN);
    reflectanceAverageLoopCounter++;
  }
  leftReflectanceThreshold = (int) (leftReflectanceThresholdSum / reflectanceAverageLoopCounter) + REFLECTANCE_THRESHOLD_OFFSET;
  rightReflectanceThreshold = (int) (rightReflectanceThresholdSum / reflectanceAverageLoopCounter) + REFLECTANCE_THRESHOLD_OFFSET;
}

void computePID() {
  error = rightReflectance - leftReflectance;
  pValue = k_p * (double) error;
  dValue = k_d * ((double) (error - prevError)) / ((double) (micros() - prevTime));
  eValue = k_e * (double) error;
  prevTime = micros();
  prevError = error;
}

void runPID(int power) {
  readReflectanceSensors();
  computePID();
  leftMotor_SetPower((int) (power + pValue + dValue - abs(eValue)));
  rightMotor_SetPower((int) (power - pValue - dValue - abs(eValue)));
}

void runHysteresis(int power) {
  leftMotor_SetPower(power + (int) (prevError * HYSTERESIS_MULTIPLIER));
  rightMotor_SetPower(power - (int) (prevError * HYSTERESIS_MULTIPLIER));
  // leftMotor_SetPower(power);
  // rightMotor_SetPower(power);
}

void rightMotor_SetPower(int power) {
  if (power > 0) {
    if (rightMotorPrev > 0) {
      rightMotorPrev = power;
      ledcWrite(RIGHT_REVERSE_CHANNEL, 0); // Unnecessary
      ledcWrite(RIGHT_FORWARDS_CHANNEL, power);
    } else if (rightMotorPrev < 0) {
      rightMotorPrev = 0;
      ledcWrite(RIGHT_FORWARDS_CHANNEL, 0);
      ledcWrite(RIGHT_REVERSE_CHANNEL, 0);
      rightTimeSwitch = micros();
    } else {
      if (micros() - rightTimeSwitch > MOTOR_SWITCH_TIME) {
        ledcWrite(RIGHT_REVERSE_CHANNEL, 0); // Unnecessary
        ledcWrite(RIGHT_FORWARDS_CHANNEL, power);
      }
    }
  } else if (power < 0) {
    if (rightMotorPrev < 0) {
      rightMotorPrev = power;
      ledcWrite(RIGHT_FORWARDS_CHANNEL, 0); // Unnecessary
      ledcWrite(RIGHT_REVERSE_CHANNEL, -power);
    } else if (rightMotorPrev > 0) {
      rightMotorPrev = 0;
      ledcWrite(RIGHT_FORWARDS_CHANNEL, 0);
      ledcWrite(RIGHT_REVERSE_CHANNEL, 0);
      rightTimeSwitch = micros();
    } else {
      if (micros() - rightTimeSwitch > MOTOR_SWITCH_TIME) {
        ledcWrite(RIGHT_FORWARDS_CHANNEL, 0); // Unnecessary
        ledcWrite(RIGHT_REVERSE_CHANNEL, -power);
      }
    }
  } else {
    if (rightMotorPrev != 0) {
      rightTimeSwitch = micros();
    }
    rightMotorPrev = 0;
    ledcWrite(RIGHT_FORWARDS_CHANNEL, 0);
    ledcWrite(RIGHT_REVERSE_CHANNEL, 0);
  }
}

void leftMotor_SetPower(int power) {
  if (power > 0) {
    if (leftMotorPrev > 0) {
      leftMotorPrev = power;
      ledcWrite(LEFT_REVERSE_CHANNEL, 0); // Unnecessary
      ledcWrite(LEFT_FORWARDS_CHANNEL, power);
    } else if (leftMotorPrev < 0) {
      leftMotorPrev = 0;
      ledcWrite(LEFT_FORWARDS_CHANNEL, 0);
      ledcWrite(LEFT_REVERSE_CHANNEL, 0);
      leftTimeSwitch = micros();
    } else {
      if (micros() - leftTimeSwitch > MOTOR_SWITCH_TIME) {
        ledcWrite(LEFT_REVERSE_CHANNEL, 0); // Unnecessary
        ledcWrite(LEFT_FORWARDS_CHANNEL, power);
      }
    }
  } else if (power < 0) {
    if (leftMotorPrev < 0) {
      leftMotorPrev = power;
      ledcWrite(LEFT_FORWARDS_CHANNEL, 0); // Unnecessary
      ledcWrite(LEFT_REVERSE_CHANNEL, -power);
    } else if (leftMotorPrev > 0) {
      leftMotorPrev = 0;
      ledcWrite(LEFT_FORWARDS_CHANNEL, 0);
      ledcWrite(LEFT_REVERSE_CHANNEL, 0);
      leftTimeSwitch = micros();
    } else {
      if (micros() - leftTimeSwitch > MOTOR_SWITCH_TIME) {
        ledcWrite(LEFT_FORWARDS_CHANNEL, 0); // Unnecessary
        ledcWrite(LEFT_REVERSE_CHANNEL, -power);
      }
    }
  } else {
    if (leftMotorPrev != 0) {
      leftTimeSwitch = micros();
    }
    leftMotorPrev = 0;
    ledcWrite(LEFT_FORWARDS_CHANNEL, 0);
    ledcWrite(LEFT_REVERSE_CHANNEL, 0);
  }
}

void verticalMotor_SetPower(int power) {
  if (power > 0) {
    if (verticalMotorPrev > 0) {
      verticalMotorPrev = power;
      ledcWrite(VERTICAL_REVERSE_CHANNEL, 0); // Unnecessary
      ledcWrite(VERTICAL_FORWARDS_CHANNEL, power);
    } else if (verticalMotorPrev < 0) {
      verticalMotorPrev = 0;
      ledcWrite(VERTICAL_FORWARDS_CHANNEL, 0);
      ledcWrite(VERTICAL_REVERSE_CHANNEL, 0);
      verticalTimeSwitch = micros();
    } else {
      if (micros() - verticalTimeSwitch > MOTOR_SWITCH_TIME) {
        ledcWrite(VERTICAL_REVERSE_CHANNEL, 0); // Unnecessary
        ledcWrite(VERTICAL_FORWARDS_CHANNEL, power);
      }
    }
  } else if (power < 0) {
    if (verticalMotorPrev < 0) {
      verticalMotorPrev = power;
      ledcWrite(VERTICAL_FORWARDS_CHANNEL, 0); // Unnecessary
      ledcWrite(VERTICAL_REVERSE_CHANNEL, -power);
    } else if (verticalMotorPrev > 0) {
      verticalMotorPrev = 0;
      ledcWrite(VERTICAL_FORWARDS_CHANNEL, 0);
      ledcWrite(VERTICAL_REVERSE_CHANNEL, 0);
      verticalTimeSwitch = micros();
    } else {
      if (micros() - verticalTimeSwitch > MOTOR_SWITCH_TIME) {
        ledcWrite(VERTICAL_FORWARDS_CHANNEL, 0); // Unnecessary
        ledcWrite(VERTICAL_REVERSE_CHANNEL, -power);
      }
    }
  } else {
    if (verticalMotorPrev != 0) {
      verticalTimeSwitch = micros();
    }
    verticalMotorPrev = 0;
    ledcWrite(VERTICAL_FORWARDS_CHANNEL, 0);
    ledcWrite(VERTICAL_REVERSE_CHANNEL, 0);
  }
}

static void writeServoRaw(Joint j, float deg) {
  deg = clampDeg(deg);
  uint16_t us  = angleToUs(j, deg);
  uint32_t duty = usToDuty(us);
  ledcWrite(JOINT_CH[j], duty);

  //updates new joint state
  joints[j].currentDeg   = deg;
  joints[j].lastUpdateMs = millis();
}

void setJointTarget(Joint j, float deg) {
  joints[j].targetDeg = clampDeg(deg);

  // If this joint is configured for “immediate”, push the command right now instead of waiting for updateServos().
  if (JOINT_SLEW_DEGPS[j] >= SPEED_IMMEDIATE_DEGPS) {
    writeServoRaw(j, joints[j].targetDeg);
  }
}

void setAllTargets(float baseDeg, float shoulderDeg, float elbowDeg, float wristDeg, float clawDeg) {
  setJointTarget(JOINT_BASE_ROT, baseDeg);
  setJointTarget(JOINT_SHOULDER, shoulderDeg);
  setJointTarget(JOINT_ELBOW,   elbowDeg);
  setJointTarget(JOINT_WRIST,   wristDeg);
  setJointTarget(JOINT_CLAW,    clawDeg);
}

void updateServos() {
  uint32_t now = millis();

  for (Joint j = 0; j < JOINT_COUNT; ++j) {
    //Obtain slew speeds
    float slew = JOINT_SLEW_DEGPS[j];
    if (slew >= SPEED_IMMEDIATE_DEGPS) continue;        // instant‑move joint

    //Compute elapsed time for this joint
    uint32_t dtMs = now - joints[j].lastUpdateMs;       // time since last rewrite PWM for this joint.
    if (dtMs == 0) continue;                            // loop raced

    joints[j].lastUpdateMs = now; //update joint state

    float currentAngle   = joints[j].currentDeg;        // last PWM angle
    float targetAngle   = joints[j].targetDeg;          // target pwm angle
    float delta = targetAngle - currentAngle;           // error
    if (fabsf(delta) < 0.01f) continue;                 // already close

    //How far this joint may move this frame (deg = slew * Δt)
    float maxStep = slew * (dtMs / 1000.0f);


    //Decides wether to snap ot step to the target angle
    if (fabsf(delta) <= maxStep) {
      writeServoRaw(j, targetAngle);                                      // snap to target
    } else {
      writeServoRaw(j, currentAngle + (delta > 0 ? maxStep : -maxStep));  // step
    }
  }
}

void waitForServos() {
  const float TOL_DEG = 0.05f;         // tolerance for knowing if a servo is still moving
  bool settling = true;
  while (settling) {
    updateServos();                    // one incremental step

    // Check if any joint is still moving
    settling = false;
    for (uint8_t j = 0; j < JOINT_COUNT; ++j) {
      if (fabsf(joints[j].currentDeg - joints[j].targetDeg) > TOL_DEG) {
        settling = true;
        break;
      }
    }
    delay(5);                         
  }
}

double computeElbowAngle(double shoulderAngle, double height) {
  return shoulderAngle + SHOULDER_OFFSET_ANGLE - ELBOW_OFFSET_ANGLE
  - asin((BICEP_LENGTH * sin(PI / 180.0 * (180 - shoulderAngle - SHOULDER_OFFSET_ANGLE)) - height) / FOREARM_LENGTH) * 180.0 / PI;
}

double computeWristAngle(double shoulderAngle, double elbowAngle, double height) {
  return 180 - shoulderAngle - SHOULDER_OFFSET_ANGLE + elbowAngle + ELBOW_OFFSET_ANGLE - WRIST_OFFSET_ANGLE;
}
