// --- Header Files --- //
// Header Files - External
#include <Arduino.h>
#include "driver/ledc.h"
#include <math.h>

// Header Files - Internal
#include "esp_constants.h"

// --- Constants --- //
const int SERVO_PWM_FREQUENCY = 50; // in Hz
const int SERRVO_PWM_NUM_BITS = 16;
const int SERVO_PWM_BITS = (1 << SERRVO_PWM_NUM_BITS) - 1;
const double SERVO_PWM_PERIOD = 1000 / SERVO_PWM_FREQUENCY; // in ms
const double MIN_MS = 1.0;
const double MAX_MS = 2.0;
const double SERVO_ERROR = 0.01;

// --- Variables --- //
struct Servo { 
  const int SERVO_PIN;
  const int SERVO_CHANNEL;
  double currentAngle; // in degrees
  double targetAngle; // in degrees
  double speed; // in degrees/ms
  unsigned long timeUpdated; // in us
};

extern Servo baseServo;
extern Servo shoulderServo;
extern Servo elbowServo;
extern Servo wristServo;
extern Servo clawServo;

// --- Functions --- //

void setServoTarget(Servo *servo, double angle) {
  servo->targetAngle = angle;
}

void updateServo(Servo *servo) {
  if (!servoDone(servo)) {
    int deltaTime = millis() - servo->timeUpdated;
    int remainingAngle = servo->targetAngle - servo->currentAngle;
    double nextAngleChange = servo->speed * deltaTime;

    if (remainingAngle < nextAngleChange) {
      servoGoTo(servo, servo->targetAngle);
      return;
    }

    servoGoTo(servo, servo->currentAngle + nextAngleChange);
  }
}

bool servoDone(Servo *servo) {
  return (fabs(servo->currentAngle - servo->targetAngle) < SERVO_ERROR);
}

double angleToPWM(double angle) {
    return (MIN_MS + (MAX_MS - MIN_MS) * (angle / 180.0)) / SERVO_PWM_PERIOD * SERVO_PWM_BITS;
}

void servoGoTo(Servo *servo, double angle) {
  ledcWrite(servo->SERVO_CHANNEL, angleToPWM(angle));
  servo->targetAngle = angle;
  servo->currentAngle = angle;
  servo->timeUpdated = millis();
}