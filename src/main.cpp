// --- Header Files --- //
#include <Arduino.h>
#include "driver/adc.h"
#include "driver/ledc.h"

// --- Pins --- //
// Pins - Motors
#define LEFT_MOTOR_FORWARDS_PIN 25
#define LEFT_MOTOR_REVERSE_PIN 26
#define RIGHT_MOTOR_FORWARDS_PIN 33
#define RIGHT_MOTOR_REVERSE_PIN 32
#define VERTICAL_MOTOR_FORWARDS_PIN 12
#define VERTICAL_MOTOR_REVERSE_PIN 13

// Pins - Servos
#define CLAW_SERVO_PIN 14
#define WRIST_SERVO_PIN 15
#define ELBOW_SERVO_PIN 2
#define LEFT_SHOULDER_SERVO_PIN 4
#define RIGHT_SHOULDER_SERVO_PIN 36 // May only need one
#define ARM_ROTATION_SERVO_PIN 39

// Pins - Sensors
#define LEFT_REFLECTANCE_PIN ADC1_CHANNEL_2 // 38
#define RIGHT_REFLECTANCE_PIN ADC1_CHANNEL_1 // 37
#define LEFT_MAGNOMETER_PIN 21
#define RIGHT_MAGNOMETER_PIN 22

// Pins - Control
#define STATE_SWITCH_PIN 5
#define STATE_INDICATOR_LED_PIN 7

// --- Channels --- //
// Channels - Drive Motors
#define LEFT_FORWARDS_CHANNEL 0
#define LEFT_REVERSE_CHANNEL 1
#define RIGHT_FORWARDS_CHANNEL 2
#define RIGHT_REVERSE_CHANNEL 3
#define VERTICAL_FORWARDS_CHANNEL 4
#define VERTICAL_REVERSE_CHANNEL 5

// Channels - Arm Servos
#define CLAW_CHANNEL 6
#define WRIST_CHANNEL 7
#define ELBOW_CHANNEL 8
#define LEFT_SHOULDER_CHANNEL 9
#define RIGHT_SHOULDER_CHANNEL 10
#define ARM_ROTATION_CHANNEL 11

// --- Constants --- //
// Constants - General

// Constants - Tape Following
#define REFLECTANCE_THRESHOLD_OFFSET 500
#define HYSTERESIS_MULTIPLIER 10

// Constants - Motors
#define MOTOR_PWM_FREQUENCY 120 // in Hz
#define MOTOR_PWM_NUM_BITS 12
#define MOTOR_SWITCH_TIME 5 // in us

// Constants - Arm
#define ARM_PWM_FREQUENCY 50 //in Hz
#define ARM_PWM_NUM_BITS 12

// --- Variables --- //
// Variables - State
enum MasterState {Initialize, Test};
#define MASTER_STATE_ARRAY_SIZE 2
MasterState masterStateArray[MASTER_STATE_ARRAY_SIZE] = {Initialize, Test};
MasterState currentMasterState;

enum ProcedureState {TapeFollow, TapeFind};
ProcedureState currentProcedureState;

// Variables - State Switch
bool stateSwitchState;

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

// Variables - Hysteresis
bool leftOnTape;
bool rightOnTape;
bool prevLeftOnTape;
bool prevRightOnTape;
int leftReflectanceThreshold;
int rightReflectanceThreshold;

// --- Function Headers --- //
void readReflectanceSensors(); // Reads and computes hysteresis variables
void computePID();
void runPID(int power); // Power between 0 and 4095
void runHysteresis(int power);
void rightMotor_SetPower(int power); // Negative power for reverse, between -4095 and 4095
void leftMotor_SetPower(int power); // Negative power for reverse, between -4095 and 4095
void verticalMotor_SetPower(int power); // Negative power for reverse, between -4095 and 4095

// --- Setup --- //
void setup() {
  // Variables - State
  currentMasterState = masterStateArray[0];

  // Variables - State Switch
  stateSwitchState = false;

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
  k_p = 0.27;
  k_i = 0.0;
  k_d = 0.3;
  pValue = 0.0;
  iValue = 0.0;
  dValue = 0.0;
  error = 0;
  prevError = 0;
  prevTime = 0;  

  // Variables - Hysteresis
  leftOnTape = true;
  rightOnTape = true;
  prevLeftOnTape = true;
  prevRightOnTape = true;

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

  // Arm PWM Setup
  ledcSetup(CLAW_CHANNEL, ARM_PWM_FREQUENCY, ARM_PWM_NUM_BITS);
  ledcSetup(WRIST_CHANNEL, ARM_PWM_FREQUENCY, ARM_PWM_NUM_BITS);
  ledcSetup(ELBOW_CHANNEL, ARM_PWM_FREQUENCY, ARM_PWM_NUM_BITS);
  ledcSetup(LEFT_SHOULDER_CHANNEL, ARM_PWM_FREQUENCY, ARM_PWM_NUM_BITS);
  ledcSetup(RIGHT_SHOULDER_CHANNEL, ARM_PWM_FREQUENCY, ARM_PWM_NUM_BITS);
  ledcSetup(ARM_ROTATION_CHANNEL, ARM_PWM_FREQUENCY, ARM_PWM_NUM_BITS);
  ledcAttachPin(CLAW_SERVO_PIN, CLAW_CHANNEL);
  ledcAttachPin(WRIST_SERVO_PIN, WRIST_CHANNEL);
  ledcAttachPin(ELBOW_SERVO_PIN, ELBOW_CHANNEL);
  ledcAttachPin(LEFT_SHOULDER_SERVO_PIN, LEFT_SHOULDER_CHANNEL);
  ledcAttachPin(RIGHT_SHOULDER_SERVO_PIN, RIGHT_SHOULDER_CHANNEL);
  ledcAttachPin(ARM_ROTATION_SERVO_PIN, ARM_ROTATION_CHANNEL);
}

// --- Loop --- //
void loop() {
//   switch(currentMasterState) {
//     case Test:
//     switch(currentProcedureState) {
//       case TapeFollow:
//       readReflectanceSensors();
//       computePID();
//       runPID(800);
//       if (!leftOnTape && !rightOnTape) {
//         currentProcedureState = TapeFind;
//       }
//       break;

//       case TapeFind:
//       readReflectanceSensors();
//       runHysteresis(800);
//       if (leftOnTape || rightOnTape) {
//         computePID();
//         leftMotor_SetPower(0);
//         rightMotor_SetPower(0);
//         currentProcedureState = TapeFollow;
//       }
//       break;
//     }
//     break;

//     case Initialize:
//     leftReflectanceThreshold = adc1_get_raw(LEFT_REFLECTANCE_PIN) + REFLECTANCE_THRESHOLD_OFFSET;
//     rightReflectanceThreshold = adc1_get_raw(RIGHT_REFLECTANCE_PIN) + REFLECTANCE_THRESHOLD_OFFSET;
//     if (millis() > 500) {
//       currentMasterState = masterStateArray[1];
//       currentProcedureState = TapeFollow;
//     }
//     break;
//   } 
}

// --- Function Definitions --- //
void readReflectanceSensors() {
  leftReflectance = adc1_get_raw(LEFT_REFLECTANCE_PIN);
  rightReflectance = adc1_get_raw(RIGHT_REFLECTANCE_PIN);
  
  // Hysteresis Computation
  prevLeftOnTape = leftOnTape;
  prevRightOnTape = rightOnTape;
  leftOnTape = (leftReflectance > leftReflectanceThreshold);
  rightOnTape = (rightReflectance > rightReflectanceThreshold);
}

void computePID() {
  error = rightReflectance - leftReflectance;
  pValue = k_p * (double) error;
  dValue = k_d * ((double) (error - prevError)) / ((double) (micros() - prevTime));
  prevTime = micros();
  prevError = error;
}

void runPID(int power) {
  leftMotor_SetPower((int) (power + pValue + dValue));
  rightMotor_SetPower((int) (power - pValue - dValue));
}

void runHysteresis(int power) {
  leftMotor_SetPower(power + (int) (k_p * prevError * HYSTERESIS_MULTIPLIER));
  rightMotor_SetPower(power - (int) (k_p * prevError * HYSTERESIS_MULTIPLIER));
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
