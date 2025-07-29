// --- Header Files --- //
// Header Files - External
#include <Arduino.h>
#include "driver/ledc.h"

// Header Files - Internal
#include "esp_constants.h"

// --- Constants --- //
const int SERVO_PWM_FREQUENCY = 50; // in Hz
const int SERRVO_PWM_NUM_BITS = 16;
const int SERVO_PWM_BITS = (1 << SERRVO_PWM_NUM_BITS) - 1;
const double SERVO_PWM_PERIOD = 1000 / SERVO_PWM_FREQUENCY; // in ms
const double MIN_MS = 1.0;
const double MAX_MS = 2.0;

// --- Variables --- //
struct Servo { 
  const int SERVO_PIN;
  const int SERVO_CHANNEL;
  int currentAngle; // in degrees
  int targetAngle; // in degrees
  int speed; // in degrees/s
  unsigned long timeUpdated; // in us
};

extern Servo baseServo;
extern Servo shoulderServo;
extern Servo elbowServo;
extern Servo wristServo;
extern Servo clawServo;

// --- Functions --- //



double angleToDutyCycle(double angle) {
    return (MIN_MS + (MAX_MS - MIN_MS) * (angle / 180.0)) / SERVO_PWM_PERIOD * SERVO_PWM_BITS;
}
