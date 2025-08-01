// --- Header Files --- //
// Header Files - External
#include <Arduino.h>
#include "driver/adc.h"
#include "driver/ledc.h"
#include <math.h>
#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

// Header Files - Internal
#include "esp_constants.h"
#include "arm.h"
#include "drivetrain.h"
#include "sensors.h"
#include "states.h"

// --- Variables --- //
// Variables - Motors
Motor leftMotor = {
    LEFT_MOTOR_FORWARDS_PIN,
    LEFT_MOTOR_REVERSE_PIN,
    LEFT_FORWARDS_CHANNEL,
    LEFT_REVERSE_CHANNEL,
    0,
    0
};
Motor rightMotor = {
    RIGHT_MOTOR_FORWARDS_PIN,
    RIGHT_MOTOR_REVERSE_PIN,
    RIGHT_FORWARDS_CHANNEL,
    RIGHT_REVERSE_CHANNEL,
    0,
    0
};
Motor verticalMotor = {
    VERTICAL_MOTOR_FORWARDS_PIN,
    VERTICAL_MOTOR_REVERSE_PIN,
    VERTICAL_FORWARDS_CHANNEL,
    VERTICAL_REVERSE_CHANNEL,
    0,
    0
};

// Variables - Reflectance Sensor
int leftReflectance;
int rightReflectance;
int forwardLeftReflectance;
int forwardRightReflectance; 

// Variables - Reflectance Sensor Initialization
int leftReflectanceThreshold;
int leftReflectanceThresholdSum;
int rightReflectanceThreshold;
int rightReflectanceThresholdSum;
int forwardLeftReflectanceThreshold;
int forwardLeftReflectanceThresholdSum;
int forwardRightReflectanceThreshold;
int forwardRightReflectanceThresholdSum;
int reflectanceAverageLoopCounter;

// Variables - Magnetometer
Adafruit_LIS3MDL lis;
double magnetometerMagnitude;

// Variables - Magnetometer Average Calculations
double magnetometerMagnitudeSum;
int magnetometerAverageCount;

// Variables - Time of Flight
Adafruit_VL6180X tof = Adafruit_VL6180X();
SemaphoreHandle_t i2cMutex;
uint16_t volatile timeOfFlightReading;

// Variables - Servos
Servo baseServo = {
    BASE_SERVO_PIN,
    BASE_SERVO_CHANNEL,
    90.0,
    90.0,
    SERVO_STARTING_SPEED,
    0
};
Servo shoulderServo = {
    SHOULDER_SERVO_PIN,
    SHOULDER_SERVO_CHANNEL,
    90.0,
    90.0,
    SERVO_STARTING_SPEED,
    0
};
Servo elbowServo = {
    ELBOW_SERVO_PIN,
    ELBOW_SERVO_CHANNEL,
    90.0,
    90.0,
    SERVO_STARTING_SPEED,
    0
};
Servo wristServo = {
    WRIST_SERVO_PIN,
    WRIST_SERVO_CHANNEL,
    90.0,
    90.0,
    SERVO_STARTING_SPEED,
    0
};
Servo clawServo = {
    CLAW_SERVO_PIN,
    CLAW_SERVO_CHANNEL,
    90.0,
    90.0,
    SERVO_STARTING_SPEED,
    0
};
Servo servoArray[NUM_SERVOS] = {
    baseServo,
    shoulderServo,
    elbowServo,
    wristServo,
    clawServo
};

// Variables - Switches
bool initializeSwitchState;
bool resetSwitchState;

// Variables - States
SwitchState currentSwitchState;
PetState currentPetState;
TaskState currentTaskState;

StepState_PrePet currentStepState_PrePet;

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
double hysteresisMultiplier;

// Variables - Hysteresis
bool leftOnTape;
bool rightOnTape;
bool prevLeftOnTape;
bool prevRightOnTape;

// Variables - Runtime
int loopCounter;
int timeCheckpoint;

// --- Function Headers --- //
void initializeState();
void resetState();
void offState();
void updateSwitchState();
void runPID_withBackup(int power);
void runPID_withHysteresis(int power);
void setPIDValues(double p, double d, double hysteresis, double e);
void pidSetup();
void computePID();
void runHysteresis(int power);

void setup() {
  drivetrainSetup();
  sensorSetup();
  armSetup();
  pidSetup();
  stateSetup();

  currentSwitchState = SwitchState::Off;
  currentPetState = PetState::PrePet;

  currentStepState_PrePet = StepState_PrePet::ClearDoorway;

  currentTaskState = TaskState::TapeFollow;
  setPIDValues(1.0, 0.0, 0.0, 0.0);

  Serial.begin(115200);
}

void loop() {
  timeCheckpoint = micros();
  switch(currentSwitchState) {

    // --- Begin Run --- //
    case SwitchState::Run:
    switch(currentPetState) {

      // --- Begin PrePet --- //
      case PetState::PrePet:
      switch(currentStepState_PrePet) {

        // --- Begin ClearDoorway --- //
        case StepState_PrePet::ClearDoorway:
        runPID_withBackup(700);
        if (timeOfFlightReading < 40) {
          drivetrainSetPower(0);
          delay(1000);
        }
        break;
        // readReflectanceSensors();
        // Serial.printf("%d | %d | %d | %d\n", leftReflectance, rightReflectance, leftReflectanceThreshold, rightReflectanceThreshold);
        // delay(200);
        // --- End ClearDoorway --- //

      }
      break;
      // --- End PrePet --- //

    }
    break;
    // --- End Run --- //

    // --- Other Switch Cases --- //
    case SwitchState::Initialize:
    initializeState();
    break;

    case SwitchState::Reset:
    resetState();
    break;

    case SwitchState::Off:
    offState();
    break;
  }
}

// --- Functions --- //
void initializeState() {
  // Reflectance initialized upon entering state
  leftMotorSetPower(0);
  rightMotorSetPower(0);
  verticalMotorSetPower(0);
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoTarget(&servoArray[i], SERVO_STARTING_ANGLES[i]);
    updateServo(&servoArray[i]);
  }
  updateSwitchState();
}

void resetState() {
  leftMotorSetPower(0);
  rightMotorSetPower(0);
  verticalMotorSetPower(-2000);
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoTarget(&servoArray[i], SERVO_STARTING_ANGLES[i]);
    updateServo(&servoArray[i]);
  }
  updateSwitchState();
}

void offState() {
  leftMotorSetPower(0);
  rightMotorSetPower(0);
  verticalMotorSetPower(0);
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoTarget(&servoArray[i], SERVO_STARTING_ANGLES[i]);
    updateServo(&servoArray[i]);
  }
  updateSwitchState();
}

void updateSwitchState() {
    // Read switch states
    initializeSwitchState = !((bool) digitalRead(INITIALIZE_SWITCH_PIN));
    resetSwitchState = !((bool) digitalRead(RESET_SWITCH_PIN));

    // Compute states
    if (initializeSwitchState && resetSwitchState) {
      currentSwitchState = SwitchState::Run;
      loopCounter = 0;
      timeCheckpoint = millis();
    } else if (initializeSwitchState && !resetSwitchState) {
      if (currentSwitchState != SwitchState::Initialize) {
        initializeReflectanceSensors(500);
      }
      currentSwitchState = SwitchState::Initialize;
    } else if (resetSwitchState) {
      currentSwitchState = SwitchState::Reset;
    } else {
      currentSwitchState = SwitchState::Off;
    }
}

void runPID_withBackup(int power) {
  switch(currentTaskState) {
    case TaskState::TapeFollow:
    readReflectanceSensors();
    computePID();
    leftMotorSetPower((int) (power + pValue + dValue - abs(eValue)));
    rightMotorSetPower((int) (power - pValue - dValue - abs(eValue)));
    if (!leftOnTape && !rightOnTape) {
      currentTaskState = TaskState::TapeFind;
    }
    break;

    case TaskState::TapeFind:
    readReflectanceSensors();
    drivetrainSetPower(-700);
    if (leftOnTape && rightOnTape) {
      computePID();
      currentTaskState = TaskState::TapeFollow;
    }
    break; 
  }
}

void runPID_withHysteresis(int power) {
  switch(currentTaskState) {
    case TaskState::TapeFollow:
    readReflectanceSensors();
    computePID();
    leftMotorSetPower((int) (power + pValue + dValue - abs(eValue)));
    rightMotorSetPower((int) (power - pValue - dValue - abs(eValue)));
    if (!leftOnTape && !rightOnTape) {
      currentTaskState = TaskState::TapeFind;
    }
    break;

    case TaskState::TapeFind:
    readReflectanceSensors();
    runHysteresis(power);
    if (leftOnTape && rightOnTape) {
      computePID();
      currentTaskState = TaskState::TapeFollow;
    }
    break; 
  }
}

void setPIDValues(double p, double d, double hysteresis, double e) {
  k_p = p;
  k_d = d;
  hysteresisMultiplier = hysteresis;
  k_e = e;
}

void pidSetup() {
  pValue = 0.0;
  iValue = 0.0;
  dValue = 0.0;
  error = 0;
  prevError = 0;
  prevTime = 0;
  eValue = 0.0;
  leftOnTape = true;
  rightOnTape = true;
  prevLeftOnTape = true;
  prevRightOnTape = true;
}

void computePID() {
  error = rightReflectance - leftReflectance;
  pValue = k_p * (double) error;
  dValue = k_d * ((double) (error - prevError)) / ((double) (micros() - prevTime));
  eValue = k_e * (double) error;
  prevTime = micros();
  prevError = error;
}

void runHysteresis(int power) {
  leftMotorSetPower(power + (int) (prevError * hysteresisMultiplier));
  rightMotorSetPower(power - (int) (prevError * hysteresisMultiplier));
}
