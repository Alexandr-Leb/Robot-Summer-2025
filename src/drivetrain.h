// --- Header Files --- //
// Header Files - External
#include <Arduino.h>
#include "driver/ledc.h"

// Header Files - Internal
#include "esp_constants.h"

// --- Constants --- //
const int MOTOR_PWM_FREQUENCY = 120; // in Hz
const int MOTOR_PWM_NUM_BITS = 12;
const int MOTOR_PWM_BITS = pow(2, MOTOR_PWM_NUM_BITS);
const int MOTOR_SWITCH_TIME = 5; // in us

// --- Variables --- //
struct Motor {
  const int FORWARDS_PIN;
  const int REVERSE_PIN;
  const int FORWARDS_CHANNEL;
  const int REVERSE_CHANNEL;
  int prevDutyCycle; // percent
  unsigned long timeSwitch; // in us
};

extern Motor leftMotor;
extern Motor rightMotor;
extern Motor verticalMotor;
 
// --- Functions --- //
void drivetrainSetup() {
  motorSetup(&leftMotor);
  motorSetup(&rightMotor);
  motorSetup(&verticalMotor);
}

void motorSetup(Motor *motor) {
  // Motor Variables Initialization
  motor->prevDutyCycle = 0;
  motor->timeSwitch = 0;
  
  // Motor PWM Setup
  ledcSetup(motor->FORWARDS_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcSetup(motor->REVERSE_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_NUM_BITS);
  ledcAttachPin(motor->FORWARDS_PIN, motor->FORWARDS_CHANNEL);
  ledcAttachPin(motor->REVERSE_PIN, motor->REVERSE_CHANNEL);
}

void leftMotorSetPower(int dutyCycle) {
  motorSetPower(&leftMotor, dutyCycle);
}

void rightMotorSetPower(int dutyCycle) {
  motorSetPower(&rightMotor, dutyCycle);
}

void verticalMotorSetPower(int dutyCycle) {
  motorSetPower(&verticalMotor, dutyCycle);
}


void motorSetPower(Motor *motor, int dutyCycle) {
  int power = (int) (dutyCycle * pow(2, MOTOR_PWM_NUM_BITS) / 100.0);
  if (dutyCycle > 0) {
    if (motor->prevDutyCycle > 0) {
      motor->prevDutyCycle = dutyCycle;
      ledcWrite(motor->REVERSE_CHANNEL, 0); // Unnecessary
      ledcWrite(motor->FORWARDS_CHANNEL, power); 
    } else if (motor->prevDutyCycle < 0) {
      motor->prevDutyCycle = 0;
      ledcWrite(motor->FORWARDS_CHANNEL, 0);
      ledcWrite(motor->REVERSE_CHANNEL, 0);
      motor->timeSwitch = micros();
    } else {
      if (micros() - motor->timeSwitch > MOTOR_SWITCH_TIME) {
        ledcWrite(motor->REVERSE_CHANNEL, 0); // Unnecessary
        ledcWrite(motor->FORWARDS_CHANNEL, power);
      }
    }
  } else if (dutyCycle < 0) {
    if (motor->prevDutyCycle < 0) {
      motor->prevDutyCycle = dutyCycle;
      ledcWrite(motor->FORWARDS_CHANNEL, 0); // Unnecessary
      ledcWrite(motor->REVERSE_CHANNEL, -power);
    } else if (motor->prevDutyCycle > 0) {
      motor->prevDutyCycle = 0;
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
    if (motor->prevDutyCycle != 0) {
      motor->timeSwitch = micros();
    }
    motor->prevDutyCycle = 0;
    ledcWrite(motor->FORWARDS_CHANNEL, 0);
    ledcWrite(motor->REVERSE_CHANNEL, 0);
  }
}
