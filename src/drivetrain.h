// --- Header Files --- //
// Header Files - External
#include <Arduino.h>
#include "driver/ledc.h"

// Header Files - Internal
#include "esp_constants.h"

// --- Constants --- //
const int MOTOR_PWM_FREQUENCY = 120; // in Hz
const int MOTOR_PWM_NUM_BITS = 12;
const int MOTOR_PWM_BITS = (1 << MOTOR_PWM_NUM_BITS) - 1;
const int MOTOR_SWITCH_TIME = 5; // in us

// --- Variables --- //
struct Motor {
  const int FORWARDS_PIN;
  const int REVERSE_PIN;
  const int FORWARDS_CHANNEL;
  const int REVERSE_CHANNEL;
  int prevPower;
  unsigned long timeSwitch; // in us
};

extern Motor leftMotor;
extern Motor rightMotor;
extern Motor verticalMotor;
 
// --- Function Headers --- //
void drivetrainSetup();
void motorSetup(Motor *motor);
void leftMotorSetPower(int power);
void rightMotorSetPower(int power);
void verticalMotorSetPower(int power);
void motorSetPower(Motor *motor, int power);
void drivetrainSetPower(int power);

// --- Functions --- //
void drivetrainSetup() {
  motorSetup(&leftMotor);
  motorSetup(&rightMotor);
  motorSetup(&verticalMotor);
}

void motorSetup(Motor *motor) {
  // Motor PWM Setup
  ledcSetup(motor->FORWARDS_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcSetup(motor->REVERSE_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcAttachPin(motor->FORWARDS_PIN, motor->FORWARDS_CHANNEL);
  ledcAttachPin(motor->REVERSE_PIN, motor->REVERSE_CHANNEL);

  // Motor Variables Initialization
  motor->prevPower = 0;
  motor->timeSwitch = 0;
}

void leftMotorSetPower(int power) {
  motorSetPower(&leftMotor, power);
}

void rightMotorSetPower(int power) {
  motorSetPower(&rightMotor, power);
}

void verticalMotorSetPower(int power) {
  motorSetPower(&verticalMotor, power);
}


void motorSetPower(Motor *motor, int power) {
  if (power > 0) {
    if (motor->prevPower > 0) {
      motor->prevPower = power;
      ledcWrite(motor->REVERSE_CHANNEL, 0); // Unnecessary
      ledcWrite(motor->FORWARDS_CHANNEL, power); 
    } else if (motor->prevPower < 0) {
      motor->prevPower = 0;
      ledcWrite(motor->FORWARDS_CHANNEL, 0);
      ledcWrite(motor->REVERSE_CHANNEL, 0);
      motor->timeSwitch = micros();
    } else {
      if (micros() - motor->timeSwitch > MOTOR_SWITCH_TIME) {
        ledcWrite(motor->REVERSE_CHANNEL, 0); // Unnecessary
        ledcWrite(motor->FORWARDS_CHANNEL, power);
      }
    }
  } else if (power < 0) {
    if (motor->prevPower < 0) {
      motor->prevPower = power;
      ledcWrite(motor->FORWARDS_CHANNEL, 0); // Unnecessary
      ledcWrite(motor->REVERSE_CHANNEL, -power);
    } else if (motor->prevPower > 0) {
      motor->prevPower = 0;
      ledcWrite(motor->FORWARDS_CHANNEL, 0);
      ledcWrite(motor->REVERSE_CHANNEL, 0);
      motor->timeSwitch = micros();
    } else {
      if (micros() - motor->timeSwitch > MOTOR_SWITCH_TIME) {
        ledcWrite(motor->FORWARDS_CHANNEL, 0); // Unnecessary
        ledcWrite(motor->REVERSE_CHANNEL, -power);
      }
    }
  } else {
    if (motor->prevPower != 0) {
      motor->timeSwitch = micros();
    }
    motor->prevPower = 0;
    ledcWrite(motor->FORWARDS_CHANNEL, 0);
    ledcWrite(motor->REVERSE_CHANNEL, 0);
  }
}

void drivetrainSetPower(int power) {
  leftMotorSetPower(power);
  rightMotorSetPower(power);
}
