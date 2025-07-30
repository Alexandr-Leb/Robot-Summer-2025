// --- Header Files --- //
// Header Files - External
#include <Arduino.h>
#include "driver/ledc.h"
#include <math.h>

// Header Files - Internal
#include "esp_constants.h"

// --- Constants --- //
const int NUM_SERVOS = 5;
const int SERVO_PWM_FREQUENCY = 50; // in Hz
const int SERRVO_PWM_NUM_BITS = 16;
const int SERVO_PWM_BITS = (1 << SERRVO_PWM_NUM_BITS) - 1;
const double SERVO_PWM_PERIOD = 1000 / SERVO_PWM_FREQUENCY; // in ms
const double MIN_MS = 1.0; // in ms
const double MAX_MS = 2.0; // in ms
const double SERVO_ERROR = 0.01; // in degrees
const double SERVO_STARTING_ANGLES[NUM_SERVOS] = {90, 90, 90, 90, 90}; // in degrees
const double SERVO_STARTING_SPEED = 40.0; // in degrees/ms

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
extern Servo servoArray[NUM_SERVOS];

// --- Function Headers --- //
void armSetup();
void setAllServoTargets(double base, double shoulder, double elbow, double wrist, double claw);
void updateServos();
bool allServosDone();
void setServoTarget(Servo *servo, double angle);
void updateServo(Servo *servo);
bool servoDone(Servo *servo);
double angleToPWM(double angle);
void servoGoTo(Servo *servo, double angle);

// --- Functions --- //
void armSetup() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    ledcSetup(servoArray[i].SERVO_CHANNEL, SERVO_PWM_FREQUENCY, SERRVO_PWM_NUM_BITS);
    ledcAttachPin(servoArray[i].SERVO_PIN, servoArray[i].SERVO_CHANNEL);
    servoGoTo(&servoArray[i], SERVO_STARTING_ANGLES[i]);
  }
}

void setAllServoTargets(double base, double shoulder, double elbow, double wrist, double claw) {
  double targets[NUM_SERVOS] = {base, shoulder, elbow, wrist, claw};
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoTarget(&servoArray[i], targets[i]);
  }
}

void updateServos() {
  for (int i  = 0; i < NUM_SERVOS; i++) {
    updateServo(&servoArray[i]);
  }
}

bool allServosDone() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!servoDone(&servoArray[i])) {
      return false;
    }
  }
  return true;
}

void setServoTarget(Servo *servo, double angle) {
  servo->targetAngle = angle;
}

void updateServo(Servo *servo) {
  if (!servoDone(servo)) {
    int deltaTime = millis() - servo->timeUpdated; // in ms
    int remainingAngle = servo->targetAngle - servo->currentAngle; // in degrees
    double nextAngleChange = servo->speed * deltaTime; // in degrees

    if (remainingAngle < nextAngleChange) {
      servoGoTo(servo, servo->targetAngle);
    } else {
      servoGoTo(servo, servo->currentAngle + nextAngleChange);
    }
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
