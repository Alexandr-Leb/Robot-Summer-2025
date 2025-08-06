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
const double MIN_MS = 0.550; // in ms
const double MAX_MS = 2.450; // in ms
const double SERVO_ERROR = 0.01; // in degrees
const double SERVO_STARTING_ANGLES[NUM_SERVOS] = {94, 50, 110, 180, 130}; // in degrees
const double SERVO_STARTING_SPEED = 0.12; // in degrees/ms
const double SERVO_MAX_ANGLE_CHANGE = 2.0; // in degrees
const double BICEP_LENGTH = 132.446; // in mm
const double FOREARM_LENGTH = 105.641; // in mm
const double SHOULDER_OFFSET_ANGLE = -7.7; // in deg rees
const double ELBOW_OFFSET_ANGLE = -16.8; // in degrees
const double WRIST_OFFSET_ANGLE = -30.6; // in degrees // Positive brings down // was -37.6
const double SHOULDER_OFFSET_ANGLE_HIGHER = -7.7; // in deg rees
const double ELBOW_OFFSET_ANGLE_HIGHER = -16.8; // in degrees
const double WRIST_OFFSET_ANGLE_HIGHER = -20.6; // in degrees // Positive brings down // was -37.6

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
extern Servo shoulderServo; // Max 170
extern Servo elbowServo;
extern Servo wristServo;
extern Servo clawServo; // Down is open
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
double calculateElbowAngle(double shoulderAngle, double height);
double calculateWristAngle(double shoulderAngle, double elbowAngle, double height);

// --- Functions --- //
void armSetup() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    ledcSetup(servoArray[i].SERVO_CHANNEL, SERVO_PWM_FREQUENCY, SERRVO_PWM_NUM_BITS);
    ledcAttachPin(servoArray[i].SERVO_PIN, servoArray[i].SERVO_CHANNEL);
    servoGoTo(&servoArray[i], SERVO_STARTING_ANGLES[i]);
  }
}

void setAllServoTargets(double base, double shoulder, double elbow, double wrist, double claw) {
  baseServo.targetAngle = base;
  shoulderServo.targetAngle = shoulder;
  elbowServo.targetAngle = elbow;
  wristServo.targetAngle = wrist;
  clawServo.targetAngle = claw;
}

void updateServos() {
  updateServo(&baseServo);
  updateServo(&shoulderServo);
  updateServo(&elbowServo);
  updateServo(&wristServo);
  updateServo(&clawServo);
}

bool allServosDone() {
  return servoDone(&baseServo)
  && servoDone(&shoulderServo)
  && servoDone(&elbowServo)
  && servoDone(&wristServo)
  && servoDone(&clawServo);
}

void setServoTarget(Servo *servo, double angle) {
  servo->targetAngle = angle;
}

void updateServo(Servo *servo) {
  if (!servoDone(servo)) {
    double servoTargetAngle = servo->targetAngle;

    int deltaTime = millis() - servo->timeUpdated; // in ms
    if (deltaTime == 0) {
      return;
    }
    double remainingAngle = servo->targetAngle - servo->currentAngle; // in degrees
    double nextAngleChange = fmin(servo->speed * deltaTime, SERVO_MAX_ANGLE_CHANGE); // in degrees
    if (remainingAngle < 0) {
      nextAngleChange *= -1.0;
    }

    if (fabs(remainingAngle) < fabs(nextAngleChange)) {
      servoGoTo(servo, servo->targetAngle);
    } else {
      servoGoTo(servo, servo->currentAngle + nextAngleChange);
    }

    servo->targetAngle = servoTargetAngle;
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

double calculateElbowAngle(double shoulderAngle, double height) {
  return shoulderAngle + SHOULDER_OFFSET_ANGLE - ELBOW_OFFSET_ANGLE
  + (180.0 / PI) * asin((BICEP_LENGTH * sin((PI / 180.0) * (shoulderAngle + SHOULDER_OFFSET_ANGLE)) - height) / FOREARM_LENGTH);
}

double calculateWristAngle(double shoulderAngle, double elbowAngle, double height) {
  return -shoulderAngle - SHOULDER_OFFSET_ANGLE + elbowAngle + ELBOW_OFFSET_ANGLE - WRIST_OFFSET_ANGLE;
}

double calculateElbowAngle_HIGHER(double shoulderAngle, double height) {
  return shoulderAngle + SHOULDER_OFFSET_ANGLE_HIGHER - ELBOW_OFFSET_ANGLE_HIGHER
  + (180.0 / PI) * asin((BICEP_LENGTH * sin((PI / 180.0) * (shoulderAngle + SHOULDER_OFFSET_ANGLE_HIGHER)) - height) / FOREARM_LENGTH);
}

double calculateWristAngle_HIGHER(double shoulderAngle, double elbowAngle, double height) {
  return -shoulderAngle - SHOULDER_OFFSET_ANGLE_HIGHER + elbowAngle + ELBOW_OFFSET_ANGLE_HIGHER - WRIST_OFFSET_ANGLE_HIGHER;
}