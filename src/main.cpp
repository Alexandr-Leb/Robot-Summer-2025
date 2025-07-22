// Git edit test
#include <Arduino.h>
#include "driver/adc.h"
#include "driver/ledc.h"

// Constants
#define LOOP_RESET_COUNT 1000
#define MOTOR_PWM_FREQUENCY 120
#define MOTOR_PWM_NUM_BITS 12
#define MOTOR_SWITCH_TIME 5 // in us
#define TAPE_FOLLOW_SPEED 800 // Between 0 and 4095
#define TAPE_FIND_SPEED 800 // Between 0 and 4095
#define REFLECTANCE_THRESHOLD_OFFSET 500

// Pins
#define REFLECTIVE_R_PIN ADC1_CHANNEL_0
#define REFLECTIVE_L_PIN ADC1_CHANNEL_3 
#define RIGHT_MOTOR_FORWARDS_PIN 4
#define RIGHT_MOTOR_BACKWARDS_PIN 13
#define LEFT_MOTOR_FORWARDS_PIN 32
#define LEFT_MOTOR_BACKWARDS_PIN 33
#define POT1_PIN ADC1_CHANNEL_1
#define POT2_PIN ADC1_CHANNEL_2
#define SWITCH 21

// Channels
#define RIGHT_FORWARDS_CHANNEL 0
#define LEFT_FORWARDS_CHANNEL 1
#define RIGHT_BACKWARDS_CHANNEL 2
#define LEFT_BACKWARDS_CHANNEL 3

// Variables
// Variables - General
int loopCounter;
enum state {test, standby, tapeFollow, tapeFind};
state currentState;

// Reflectance Sensor Variables
int reflective_R;
int reflective_L;

// Motor Control Variables
int rightMotorPrev;
int leftMotorPrev;
unsigned long rightTimeSwitch; // in us
unsigned long leftTimeSwitch; // in us

// PID Variables
double k_p;
double k_i; // Not using
double k_d;
double pValue;
double iValue; // Not using
double dValue;
int error;
int prevError;
long prevTime; // in us

// Hysteresis Variables
bool onTape_L;
bool onTape_R;
bool prevOnTape_L;
bool prevOnTape_R;
int rightReflectanceThreshold;
int leftReflectanceThreshold;

// Input Variables
bool switchState;

// Function Definitions
void readSensors(); // Reads sensors and computes some data
void rightMotor_SetPower(int power); // Negative power for reverse, between -4095 and 4095
void leftMotor_SetPower(int power); // Negative power for reverse, between -4095 and 4095
void computePID();

void setup() {
  // Variables
  loopCounter= 0;
  currentState = standby;
  rightMotorPrev = 0;
  leftMotorPrev = 0;
  rightTimeSwitch = 0;
  leftTimeSwitch = 0;
  k_p = 1.0;
  k_i = 0.0;
  k_d = 0.0;
  pValue = 0.0;
  iValue = 0.0;
  dValue = 0.0;
  error = 0;
  prevError = 0;
  prevTime = 0;
  onTape_L = true;
  onTape_R = true;
  prevOnTape_L = true;
  prevOnTape_R = true;

  // Reflective Analog Input Setup
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(REFLECTIVE_L_PIN, ADC_ATTEN_DB_12);
  adc1_config_channel_atten(REFLECTIVE_R_PIN, ADC_ATTEN_DB_12);

  // Motor PWM Setup
  ledcSetup(RIGHT_FORWARDS_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcSetup(LEFT_FORWARDS_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcSetup(RIGHT_BACKWARDS_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcSetup(LEFT_BACKWARDS_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcAttachPin(RIGHT_MOTOR_FORWARDS_PIN, RIGHT_FORWARDS_CHANNEL);
  ledcAttachPin(LEFT_MOTOR_FORWARDS_PIN, LEFT_FORWARDS_CHANNEL);
  ledcAttachPin(RIGHT_MOTOR_BACKWARDS_PIN, RIGHT_BACKWARDS_CHANNEL);
  ledcAttachPin(LEFT_MOTOR_BACKWARDS_PIN, LEFT_BACKWARDS_CHANNEL);

  // Potentiometer Setup
  adc1_config_channel_atten(POT1_PIN, ADC_ATTEN_DB_12);
  adc1_config_channel_atten(POT2_PIN, ADC_ATTEN_DB_12);

  // Serial Setup
  Serial.begin(115200);

  // Switch Setup
  pinMode(SWITCH, INPUT_PULLUP);
  switchState = true;
}

void loop() {
  switch(currentState) {
    case test:
      // Test Code
      break;
    case standby:
      // Standby Code
      readSensors();
      k_p = adc1_get_raw(POT1_PIN) / 2048.0;
      k_d = adc1_get_raw(POT2_PIN) / 200.0;

      leftReflectanceThreshold = reflective_L + REFLECTANCE_THRESHOLD_OFFSET;
      rightReflectanceThreshold = reflective_R + REFLECTANCE_THRESHOLD_OFFSET;

      switchState = digitalRead(SWITCH);

      if (loopCounter == 0) {
        Serial.printf("%.3lf | %.3lf\n", k_p, k_d);
      }

      if (!switchState) {
        delay(2000);
        currentState = tapeFollow;
      }

      break;
    case tapeFollow:
      // Tape Follow Code
      readSensors();
      computePID();
      if (onTape_L || onTape_R) {
        leftMotor_SetPower(TAPE_FOLLOW_SPEED + pValue + dValue);
        rightMotor_SetPower(TAPE_FOLLOW_SPEED - pValue - dValue);
      } else {
        currentState = tapeFind;
      }
      break;
    case tapeFind:
      // Tape Finding Code
      readSensors();
      leftMotor_SetPower(-TAPE_FIND_SPEED);
      rightMotor_SetPower(-TAPE_FIND_SPEED);
      if (onTape_L && onTape_R) {
        computePID();
        computePID();
        currentState = tapeFollow;
      }
      break;
  }

  loopCounter++;
  if (loopCounter == LOOP_RESET_COUNT) {
    loopCounter = 0;
  }
}

// Functions
void readSensors() {
  prevOnTape_L = onTape_L;
  prevOnTape_R = onTape_R;
  reflective_R = adc1_get_raw(REFLECTIVE_R_PIN);
  reflective_L = adc1_get_raw(REFLECTIVE_L_PIN);
  onTape_L = (reflective_L > leftReflectanceThreshold);
  onTape_R = (reflective_R > rightReflectanceThreshold);
}

void rightMotor_SetPower(int power) {
  if (power > 0) {
    if (rightMotorPrev > 0) {
      rightMotorPrev = power;
      ledcWrite(RIGHT_BACKWARDS_CHANNEL, 0); // Unnecessary
      ledcWrite(RIGHT_FORWARDS_CHANNEL, power);
    } else if (rightMotorPrev < 0) {
      rightMotorPrev = 0;
      ledcWrite(RIGHT_FORWARDS_CHANNEL, 0);
      ledcWrite(RIGHT_BACKWARDS_CHANNEL, 0);
      rightTimeSwitch = micros();
    } else {
      if (micros() - rightTimeSwitch > MOTOR_SWITCH_TIME) {
        ledcWrite(RIGHT_BACKWARDS_CHANNEL, 0); // Unnecessary
        ledcWrite(RIGHT_FORWARDS_CHANNEL, power);
      }
    }
  } else if (power < 0) {
    if (rightMotorPrev < 0) {
      rightMotorPrev = power;
      ledcWrite(RIGHT_FORWARDS_CHANNEL, 0); // Unnecessary
      ledcWrite(RIGHT_BACKWARDS_CHANNEL, -power);
    } else if (rightMotorPrev > 0) {
      rightMotorPrev = 0;
      ledcWrite(RIGHT_FORWARDS_CHANNEL, 0);
      ledcWrite(RIGHT_BACKWARDS_CHANNEL, 0);
      rightTimeSwitch = micros();
    } else {
      if (micros() - rightTimeSwitch > MOTOR_SWITCH_TIME) {
        ledcWrite(RIGHT_FORWARDS_CHANNEL, 0); // Unnecessary
        ledcWrite(RIGHT_BACKWARDS_CHANNEL, -power);
      }
    }
  } else {
    if (rightMotorPrev != 0) {
      rightTimeSwitch = micros();
    }
    rightMotorPrev = 0;
    ledcWrite(RIGHT_FORWARDS_CHANNEL, 0);
    ledcWrite(RIGHT_BACKWARDS_CHANNEL, 0);
  }
}

void leftMotor_SetPower(int power) {
  if (power > 0) {
    if (leftMotorPrev > 0) {
      leftMotorPrev = power;
      ledcWrite(LEFT_BACKWARDS_CHANNEL, 0); // Unnecessary
      ledcWrite(LEFT_FORWARDS_CHANNEL, power);
    } else if (leftMotorPrev < 0) {
      leftMotorPrev = 0;
      ledcWrite(LEFT_FORWARDS_CHANNEL, 0);
      ledcWrite(LEFT_BACKWARDS_CHANNEL, 0);
      leftTimeSwitch = micros();
    } else {
      if (micros() - leftTimeSwitch > MOTOR_SWITCH_TIME) {
        ledcWrite(LEFT_BACKWARDS_CHANNEL, 0); // Unnecessary
        ledcWrite(LEFT_FORWARDS_CHANNEL, power);
      }
    }
  } else if (power < 0) {
    if (leftMotorPrev < 0) {
      leftMotorPrev = power;
      ledcWrite(LEFT_FORWARDS_CHANNEL, 0); // Unnecessary
      ledcWrite(LEFT_BACKWARDS_CHANNEL, -power);
    } else if (leftMotorPrev > 0) {
      leftMotorPrev = 0;
      ledcWrite(LEFT_FORWARDS_CHANNEL, 0);
      ledcWrite(LEFT_BACKWARDS_CHANNEL, 0);
      leftTimeSwitch = micros();
    } else {
      if (micros() - leftTimeSwitch > MOTOR_SWITCH_TIME) {
        ledcWrite(LEFT_FORWARDS_CHANNEL, 0); // Unnecessary
        ledcWrite(LEFT_BACKWARDS_CHANNEL, -power);
      }
    }
  } else {
    if (leftMotorPrev != 0) {
      leftTimeSwitch = micros();
    }
    leftMotorPrev = 0;
    ledcWrite(LEFT_FORWARDS_CHANNEL, 0);
    ledcWrite(LEFT_BACKWARDS_CHANNEL, 0);
  }
}

void computePID() {
  error = reflective_R - reflective_L;
  pValue = k_p * (double) error;
  dValue = k_d * ((double) (error - prevError)) / ((double) (micros() - prevTime));
  prevTime = micros();
  prevError = error;
}