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

// --- Constants --- //

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
    LEFT_REVERSE_CHANNEL,
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
double timeOfFlightReading;

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

void setup() {
  Serial.begin(115200);

  drivetrainSetup();
  sensorSetup();
  armSetup();
  stateSetup();
}

void loop() {
  updateSwitchState();
  switch(currentSwitchState) {
    case SwitchState::Run:
    Serial.printf("State: Run\n");
    delay(500);
    break;

    case SwitchState::Initialize:
    Serial.printf("State: Init\n");
    readReflectanceSensors();
    readForwardReflectanceSensors();
    readMagnetometer();
    readTimeOfFlight();
    Serial.printf("LR: %d | RR: %d | Mag: %d | Tof: %d | FLR: %d | FRR: %d\n", leftReflectance, rightReflectance, magnetometerMagnitude, timeOfFlightReading, forwardLeftReflectance, forwardRightReflectance);
    delay(500);
    break;

    case SwitchState::Reset:
    Serial.printf("State: Reset\n");
    leftMotorSetPower(0);
    rightMotorSetPower(0);
    verticalMotorSetPower(-25);
    for (int i = 0; i < NUM_SERVOS; i++) {
      setServoTarget(&servoArray[i], SERVO_STARTING_ANGLES[i]);
      updateServo(&servoArray[i]);
    }
    break;

    case SwitchState::Off:
    Serial.printf("State: Off\n");
    leftMotorSetPower(0);
    rightMotorSetPower(0);
    verticalMotorSetPower(0);
    for (int i = 0; i < NUM_SERVOS; i++) {
      setServoTarget(&servoArray[i], SERVO_STARTING_ANGLES[i]);
      updateServo(&servoArray[i]);
    }
    break;
  }
}
