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
const int REFLECTANCE_PRESET_CALIBRATION = 2700;
const int REFLECTANCE_COMPARISON_THRESHOLD = 150;
const int NUM_REFLECTANCE_AVERAGE_COUNT = 10;
const int NUM_REFLECTANCE_AVERAGE_COUNT_TAPE = 5;
const int FORWARD_REFLECTANCE_COMPARISON_THRESHOLD = 500;
const int TOF_DETECTION_THRESHOLD = 35;
const int TOF_RANGE_THRESHOLD = 240;
const double GRAB_SHOULDER_ANGLE_INCREMENT = 0.2;
const int TOF_NO_SWEEP_THRESHOLD = 50;
const int TOF_DETECTION_TIME_THRESHOLD = 200;

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

// Variables - Magnetometer Maximum
double maxMagnetometerReading;
double maxMagnetometerBaseAngle;

// Variables - Time of Flight
Adafruit_VL53L1X tof(XSHUT_PIN, IRQ_PIN);
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
StepState_Pet1 currentStepState_Pet1;
StepState_Ramp currentStepState_Ramp;
StepState_Pet2 currentStepState_Pet2;
StepState_Pet3 currentStepState_Pet3;
StepState_Pet4 currentStepState_Pet4;
StepState_Pet5 currentStepState_Pet5;
StepState_Pet6 currentStepState_Pet6;
StepState_Debris currentStepState_Debris;
StepState_Pet7 currentStepState_Pet7;
StepState_PostPet currentStepState_PostPet;

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
unsigned long timeCheckpoint;
bool presetCalibration;
unsigned long breakTime;

double height;
double nextShoulderAngle;
double nextElbowAngle;
double nextWristAngle;

int leftReflectanceSum;
int reflectanceCount;

int forwardLeftReflectanceSum;
int forwardRightReflectanceSum;
int forwardReflectanceCount;

int timeOfFlightSum;
int timeOfFlightCount;
int arrayValueMark;
int lowerAngleMark;
int upperAngleMark;
int sweepAngleIncrement;
int maxTOFbaseAngle;

unsigned long basketTime;

// --- Runtime Parameters --- //
// Runtime Parameters - General
const double CLAW_CLOSED_ANGLE = 2.0;
const int CLAW_CLOSE_TIME = 1200;

// Runtime Parameters - PrePet
const int BUCKET_CLEAR_TIME = 2000;
const int GATE_CLEAR_TIME = 800;

// Runtime Parameters - Pet1
const int LIFT_BASKET_TIME = 2500;
const int PET1_GRAB_TIME = 5000;

// Runtime Parameters - Ramp
const int RAMP_DETECTION_THRESHOLD = 200;
const int RAMP_CORREECTION_TIMEOUT = 500;
const int CLIMB_TIME_BEFOE_DROP = 1000;
const int RAMP_TAPE_FOUND_THRESHOLD = 2800;
const int INCH_FORWARDS_TIME = 500;
const int TOP_RAMP_STOP_TIME = 1000;
const int VEER_LEFT_TIME = 800;
const int RAMP_NOT_FOUND_TIMEOUT = 5000;

// Runtime Parameters - Pet2
const int PET2_SLOW_DOWN_TIME = 200;

// Runtime Parameters - Pet3
const int PET3_DETECTION_TIMEOUT = 750;
const int PET3_LOWER_SWEEP_ANGLE = 90;
const int PET3_UPPER_SWEEP_ANGLE = 170;
const int PET3_SWEEP_TIME = 3000;
double pet3TimeOfFlightArray[PET3_UPPER_SWEEP_ANGLE - PET3_LOWER_SWEEP_ANGLE];

// Runtime Parameters - Pet4
const int PET4_DETECTION_TIMEOUT = 500;
const int PET4_LOWER_SWEEP_ANGLE = 130;
const int PET4_UPPER_SWEEP_ANGLE = 180;
const int PET4_SWEEP_TIME = 3000;
double pet4TimeOfFlightArray[PET4_UPPER_SWEEP_ANGLE - PET4_LOWER_SWEEP_ANGLE];

// Runtime Parameters - Pet5
const int PET5_DETECTION_TIMEOUT = 1000;
const int PET5_LOWER_SWEEP_ANGLE = 130;
const int PET5_UPPER_SWEEP_ANGLE = 180;
const int PET5_SWEEP_TIME = 3000;
double pet5TimeOfFlightArray[PET5_UPPER_SWEEP_ANGLE - PET5_LOWER_SWEEP_ANGLE];

// Runtime Parameters - Pet6

// Runtime Parameters - Debris
const int DEBRIS_FIND_TIME = 3000;
const int DEBRIS_CROSS_TIME = 1000;
const int DEBRIS_BACKUP_TIME = 2000;
const int DEBRIS_SWEEP_RIGHT_TIME = 2000;

// Runtime Parameters - PostPet
const int REFLECTANCE_EDGE_THRESHOLD = 3300;
const int REVERSE_BREAK_THRESHOLD = 500;
const int RAISE_BASKET_TIME = 6000;
const int CLEAR_ZIPLINE_TIME = 1200;

// --- Function Headers --- //
// Function Headers - Runtime Functions
void dropPetBasket();
void dislodgePet();

// Function Headers - Switch State Functions
void initializeState();
void resetState();
void offState();
void updateSwitchState();

// Function Headers - PID Functions
void runPID_withBackup(int power);
void runPID_withHysteresis(int power);
void runPID_onRamp(int power);
void setPIDValues(double p, double d, double hysteresis, double e);
void pidSetup();
void computePID();
void runHysteresis(int power);
void resetError();
void drivetrainBreak(int power);

void setup() {
  drivetrainSetup();
  sensorSetup();
  armSetup();
  pidSetup();
  stateSetup();

  presetCalibration = true;

  currentSwitchState = SwitchState::Off;
  currentPetState = PetState::PrePet;

  currentStepState_PrePet = StepState_PrePet::ClearBucket;
  currentStepState_Pet1 = StepState_Pet1::FindTarget1;
  currentStepState_Ramp = StepState_Ramp::FindRamp;
  currentStepState_Pet2 = StepState_Pet2::FindTarget2;
  currentStepState_Pet3 = StepState_Pet3::FindTarget3;
  currentStepState_Pet4 = StepState_Pet4::FindTarget4;
  currentStepState_Pet5 = StepState_Pet5::FindTarget5;
  currentStepState_Pet6 = StepState_Pet6::FindTarget6;
  currentStepState_Debris = StepState_Debris::FindDebris;
  currentStepState_Pet7 = StepState_Pet7::FindTarget7;
  currentStepState_PostPet = StepState_PostPet::FindEdge;

  currentTaskState = TaskState::TapeFollow;
  setPIDValues(2.1, 0.5, 0.0, 0.0);

  Serial.begin(115200);
}

void loop() {
  switch(currentSwitchState) {
 
    // --- Begin Run --- //
    case SwitchState::Run:
    switch(currentPetState) {

      // --- Begin PrePet --- //
      case PetState::PrePet:
      switch(currentStepState_PrePet) {

        // --- Begin ClearBucket --- //
        case StepState_PrePet::ClearBucket:
        runPID_withBackup(900);
        if (millis() - timeCheckpoint > BUCKET_CLEAR_TIME) {
          currentStepState_PrePet = StepState_PrePet::FindGate;
          setPIDValues(1.5, 0.0, 0.0, 0.0);
          timeCheckpoint = millis();
        }
        break;
        // --- End clearBucket --- //

        // --- Begin FindGate --- //
        case StepState_PrePet::FindGate:
        runPID_withBackup(900);
        if (timeOfFlightReading < 80) {
          currentStepState_PrePet = StepState_PrePet::ClearDoorway;
          setPIDValues(1.2, 0.0, 3.0, 0.0);
          timeCheckpoint = millis();
        }
        break;
        // --- End FindGate --- //

        // --- Begin ClearDoorway --- //
        case StepState_PrePet::ClearDoorway:
        runPID_withHysteresis(700);
        if (millis() - timeCheckpoint > GATE_CLEAR_TIME) {
          currentStepState_PrePet = StepState_PrePet::TurnArm;
          drivetrainSetPower(0);
          timeCheckpoint = millis();
        }
        break;
        // --- End ClearDoorway --- //

        // --- Begin TurnArm --- //
        case StepState_PrePet::TurnArm:
        resetError();
        setAllServoTargets(40, 70, 177, 137, 120);
        while (!allServosDone()) {
          updateServos();
        }
        if (allServosDone()) {
          currentPetState = PetState::Pet1;
          resetError();
          timeCheckpoint = millis();
        }
        break;
        // --- End TurnArm --- //

      }
      break;
      // --- End PrePet --- //

      // --- Begin Pet1 --- //
      case PetState::Pet1:
      switch(currentStepState_Pet1) {

        // --- Begin FindTarget1 --- //
        case StepState_Pet1::FindTarget1:
        if (millis() - timeCheckpoint > 300) {
          setPIDValues(2.1, 0.5, 0.0, 0.0);
          runPID_withBackup(900);
        } else {
          setPIDValues(1.2, 0.0, 0.0, 0.0);
          runPID_withBackup(700);
        }
        if (timeOfFlightReading < 80) { 
          currentStepState_Pet1 = StepState_Pet1::LiftBasket;
          drivetrainBreak(700);
          timeCheckpoint = millis();
        }
        break;
        // --- End FindTarget1 --- //

        // --- Begin LiftBasket --- //
        case StepState_Pet1::LiftBasket:
        verticalMotorSetPower(2000); 
        readReflectanceSensors();
        computePID();
        currentStepState_Pet1 = StepState_Pet1::ArmSearchPreset1;
        baseServo.speed = 0.02;
        if (timeOfFlightReading < TOF_NO_SWEEP_THRESHOLD) {
          currentStepState_Pet1 = StepState_Pet1::PetGrab1;
        }
        timeCheckpoint = millis();
        basketTime = millis();
        break;
        // --- End LiftBasket --- //

        // --- Begin ArmSearchPreset1 --- //
        case StepState_Pet1::ArmSearchPreset1:
        if (millis() - basketTime > LIFT_BASKET_TIME) {
          verticalMotorSetPower(0);
        }
        setServoTarget(&baseServo, 60);
        updateServo(&baseServo);
        if (servoDone(&baseServo)) {
          currentStepState_Pet1 = StepState_Pet1::PetSearch1;
          maxMagnetometerReading = 0.0;
          maxMagnetometerBaseAngle = baseServo.currentAngle;
          baseServo.speed = 0.04;
          delay(500);
        }
        if (timeOfFlightReading < TOF_NO_SWEEP_THRESHOLD) {
          currentStepState_Pet1 = StepState_Pet1::PetGrab1;
          setServoTarget(&baseServo, baseServo.currentAngle);
        }
        break;
        // --- End ArmSearchPreset1 --- //

        // --- Begin PetSearch1 --- //
        case StepState_Pet1::PetSearch1:
        if (millis() - basketTime > LIFT_BASKET_TIME) {
          verticalMotorSetPower(0);
        }
        setServoTarget(&baseServo, 10);
        updateServo(&baseServo);
        readMagnetometer();
        if (magnetometerMagnitude > maxMagnetometerReading) {
          maxMagnetometerReading = magnetometerMagnitude;
          maxMagnetometerBaseAngle = baseServo.currentAngle;
        }
        if (servoDone(&baseServo)) {
          currentStepState_Pet1 = StepState_Pet1::PetFound1;
          baseServo.speed = SERVO_STARTING_SPEED;
        }
        break;
        // --- End PetSearch1 --- //

        // --- Begin PetFound1 --- //
        case StepState_Pet1::PetFound1:
        if (millis() - basketTime > LIFT_BASKET_TIME) {
          verticalMotorSetPower(0);
        }
        setServoTarget(&baseServo, maxMagnetometerBaseAngle);
        updateServo(&baseServo);
        if (servoDone(&baseServo)) {
          currentStepState_Pet1 = StepState_Pet1::PetGrab1;
        }
        break;
        // --- End PetFound1 --- //

        // --- Begin PetGrab1 --- // 
        case StepState_Pet1::PetGrab1:
        if (millis() - basketTime > LIFT_BASKET_TIME) {
          verticalMotorSetPower(0);
        }
        if (timeOfFlightReading < TOF_DETECTION_THRESHOLD || shoulderServo.targetAngle < 5) {
          delay(TOF_DETECTION_TIME_THRESHOLD);
          if (timeOfFlightReading < TOF_DETECTION_THRESHOLD || shoulderServo.targetAngle < 5) {
            servoGoTo(&clawServo, CLAW_CLOSED_ANGLE);
            delay(CLAW_CLOSE_TIME);
            currentStepState_Pet1 = StepState_Pet1::ReturnArm1;
            baseServo.speed = SERVO_STARTING_SPEED;
            timeCheckpoint = millis();
          }
        }
        height = 12.0;
        nextShoulderAngle = shoulderServo.currentAngle - GRAB_SHOULDER_ANGLE_INCREMENT;
        nextElbowAngle = calculateElbowAngle(nextShoulderAngle, height);
        nextWristAngle = calculateWristAngle(nextShoulderAngle, nextElbowAngle, height);
        setAllServoTargets(baseServo.currentAngle, nextShoulderAngle, nextElbowAngle, nextWristAngle, clawServo.currentAngle);
        while (!allServosDone()) {
          updateServos();
          delay(20);
        }
        break;
        // --- End PetGrab1 --- //

        // --- Begin ReturnArm1 --- //
        case StepState_Pet1::ReturnArm1:
        if (millis() - basketTime > LIFT_BASKET_TIME) {
          verticalMotorSetPower(0);
        }
        setServoTarget(&wristServo, 140);
        setServoTarget(&shoulderServo, 90);
        while (!servoDone(&wristServo) || !servoDone(&shoulderServo)) {
          updateServo(&wristServo);
          updateServo(&shoulderServo);
          delay(20);
        }
        setAllServoTargets(90, 90, 160, 142, CLAW_CLOSED_ANGLE);
        while (!allServosDone()) {
          updateServos();
          delay(20);
        }
        if (allServosDone()) {
          currentPetState = PetState::Ramp;
          setPIDValues(1.2, 0.0, 0.0, 0.0);
          forwardLeftReflectanceSum = 0;
          forwardReflectanceCount = 0;
          resetError();
        }
        break;
        // --- End ReturnArm1 --- //

      }
      break;
      // --- End Pet1 --- //

      // --- Begin Ramp --- //
      case PetState::Ramp:
      switch(currentStepState_Ramp) {

        // --- Begin FindRamp --- //
        case StepState_Ramp::FindRamp:
        if (millis() - basketTime > LIFT_BASKET_TIME) {
          verticalMotorSetPower(0);
        }
        runPID_withBackup(700);
        readForwardReflectanceSensors();
        forwardLeftReflectanceSum += forwardLeftReflectance;
        forwardReflectanceCount++;
        if (forwardReflectanceCount == NUM_REFLECTANCE_AVERAGE_COUNT - 1) {
          if (forwardLeftReflectanceSum / forwardReflectanceCount < RAMP_DETECTION_THRESHOLD) {
            currentStepState_Ramp = StepState_Ramp::AlignRamp;
            timeCheckpoint = millis();
            while (millis() - timeCheckpoint < 800) {
              leftMotorSetPower(1800);
              rightMotorSetPower(0);
            }
            drivetrainSetPower(0);
            leftReflectanceSum = 0;
            reflectanceCount = 0;
            timeCheckpoint = millis();
          }
          forwardLeftReflectanceSum = 0;
          forwardReflectanceCount = 0;
        }
        if (millis() - timeCheckpoint > RAMP_NOT_FOUND_TIMEOUT) {
          currentStepState_Ramp = StepState_Ramp::AlignRamp;
            drivetrainSetPower(0);
            leftReflectanceSum = 0;
            reflectanceCount = 0;
            timeCheckpoint = millis();
        }
        break;
        // --- End FindRamp --- //

        // --- Begin AlignRamp --- //
        case StepState_Ramp::AlignRamp:
        rightMotorSetPower(1500);
        readReflectanceSensors();
        leftReflectanceSum += leftReflectance;
        reflectanceCount++;
        if (reflectanceCount == NUM_REFLECTANCE_AVERAGE_COUNT_TAPE) {
          if ((double) leftReflectanceSum / reflectanceCount > RAMP_TAPE_FOUND_THRESHOLD && millis() - timeCheckpoint > RAMP_CORREECTION_TIMEOUT) {
            currentStepState_Ramp = StepState_Ramp::DropPet1;
            setPIDValues(1.8, 0.0, 4.0, 0.0);
            resetError();
            timeCheckpoint = millis();
            while (millis() - timeCheckpoint < 500) {
              runPID_withHysteresis(900);
            }
            timeCheckpoint = millis();
            while (millis() - timeCheckpoint < CLIMB_TIME_BEFOE_DROP) {
              runPID_withHysteresis(1800); // was 1600
            }
            timeCheckpoint = millis();
            while (millis() - timeCheckpoint < 500) {
              runPID_withHysteresis(900);
            }
            resetError();
            baseServo.speed = 0.1;
            timeCheckpoint = millis();
          }
          leftReflectanceSum = 0;
          reflectanceCount = 0;
        }
        break;
        // --- End AlignRamp --- //

        // // --- Begin RampCorrection --- //
        // case StepState_Ramp::RampCorrection:
        // if (millis() - timeCheckpoint > 1000) {
        //   currentStepState_Ramp = StepState_Ramp::DropPet1;
        //   runPID_withHysteresis(900);
        //   timeCheckpoint = millis();
        // }
        // break;
        // // --- End RampCorrection --- //

        // --- Begin DropPet1 --- //
        case StepState_Ramp::DropPet1:
        runPID_withHysteresis(900);
        if (millis() - timeCheckpoint <= 1000) {
          setServoTarget(&baseServo, 170);
          updateServos();
        } else if (millis() - timeCheckpoint <= 1600) {
          setAllServoTargets(180, 40, 100, 120, CLAW_CLOSED_ANGLE);
          updateServos();
        } else if (millis() - timeCheckpoint <= 2300) {
          // servoGoTo(&baseServo, 170);
          servoGoTo(&clawServo, 120);
        } else if (millis() - timeCheckpoint <= 3200) {
          setAllServoTargets(90, 70, 177, 142, 120);
          updateServos();
        } else {
          currentStepState_Ramp = StepState_Ramp::ClimbRamp;
          readReflectanceSensors();
          while (!leftOnTape && !rightOnTape) {
            readReflectanceSensors();
            leftMotorSetPower(1600);
            rightMotorSetPower(500);
          }
          setPIDValues(1.8, 0.0, 4.0, 0.0);
          forwardLeftReflectanceSum = 0;
          forwardRightReflectanceSum = 0;
          forwardReflectanceCount = 0;
          baseServo.speed = SERVO_STARTING_SPEED;
          resetError();
          timeCheckpoint = millis();
        }
        break;
        // --- End DropPet1 --- //

        // --- Begin ClimbRamp --- //
        case StepState_Ramp::ClimbRamp:
        runPID_onRamp(1800);
        readForwardReflectanceSensors();
        forwardLeftReflectanceSum += forwardLeftReflectance;
        forwardRightReflectanceSum += forwardRightReflectance;
        forwardReflectanceCount++;
        if (forwardReflectanceCount == NUM_REFLECTANCE_AVERAGE_COUNT) {
          if ((double) forwardLeftReflectanceSum / forwardReflectanceCount > 2000.0 && (double) forwardRightReflectanceSum / forwardReflectanceCount > 3000.0) {
            currentStepState_Ramp = StepState_Ramp::InchForwards;
            timeCheckpoint = millis();
          }
          forwardLeftReflectanceSum = 0;
          forwardRightReflectanceSum = 0;
          forwardReflectanceCount = 0;
        }
        timeCheckpoint = millis();
        break;
        // --- End ClimbRamp --- //

        // --- Begin InchForwards --- //
        case StepState_Ramp::InchForwards:
        setServoTarget(&baseServo, 135);
        updateServo(&baseServo);
        runPID_withHysteresis(1400);
        if (millis() - timeCheckpoint > INCH_FORWARDS_TIME) {
          currentPetState = PetState::Pet2;
          breakTime = millis();
          while (millis() - breakTime < 200) {
            drivetrainSetPower(-1000);
          }
          drivetrainSetPower(0);
          delay(TOP_RAMP_STOP_TIME);
          setPIDValues(1.8, 0.0, 4.0, 0.0);
          resetError();
          timeCheckpoint = millis();
        }
        break;  
        // --- End InchForwards --- //

      }
      break;
      // --- End Ramp --- //

      // --- Begin Pet2 --- //
      case PetState::Pet2:
      switch(currentStepState_Pet2) {

        // --- Begin FindTarget2 --- //
        case StepState_Pet2::FindTarget2:
        if (millis() - timeCheckpoint < PET2_SLOW_DOWN_TIME) {
          setPIDValues(1.8, 0.0, 4.0, 0.0);
          runPID_withHysteresis(1200);
        } else {
          setPIDValues(2.1, 0.5, 4.0, 0.0);
          runPID_withHysteresis(900);
        }
        if (timeOfFlightReading < 70) {
          currentStepState_Pet2 = StepState_Pet2::ArmSearchPreset2;
          drivetrainBreak(1400);
          baseServo.speed = 0.02;
          if (timeOfFlightReading < TOF_NO_SWEEP_THRESHOLD) {
            currentStepState_Pet2 = StepState_Pet2::PetGrab2;
          }
          timeCheckpoint = millis();
        }
        break;
        // --- End FindTarget2 --- //

        // --- Begin ArmSearchPreset2 --- //
        case StepState_Pet2::ArmSearchPreset2:
        setServoTarget(&baseServo, 90);
        updateServo(&baseServo);
        if (servoDone(&baseServo)) {
          currentStepState_Pet2 = StepState_Pet2::PetSearch2;
          maxMagnetometerReading = 0.0;
          maxMagnetometerBaseAngle = baseServo.currentAngle;
          baseServo.speed = 0.04;
          delay(500);
          timeCheckpoint = millis();
        }
        if (timeOfFlightReading < TOF_NO_SWEEP_THRESHOLD) {
          currentStepState_Pet2 = StepState_Pet2::PetGrab2;
          setServoTarget(&baseServo, baseServo.currentAngle);
          timeCheckpoint = millis();
        }
        break;
        // --- End ArmSearchPreset2 --- //

        // --- Begin PetSearch2 --- //
        case StepState_Pet2::PetSearch2:
        setServoTarget(&baseServo, 150);
        updateServo(&baseServo);
        readMagnetometer();
        if (magnetometerMagnitude > maxMagnetometerReading) {
          maxMagnetometerReading = magnetometerMagnitude;
          maxMagnetometerBaseAngle = baseServo.currentAngle;
        }
        if (servoDone(&baseServo)) {
          currentStepState_Pet2 = StepState_Pet2::PetFound2;
          baseServo.speed = SERVO_STARTING_SPEED;
          timeCheckpoint = millis();
        }
        break;
        // --- End PetSearch2 --- //

        // --- Begin PetFound2 --- //
        case StepState_Pet2::PetFound2:
        setServoTarget(&baseServo, maxMagnetometerBaseAngle);
        updateServo(&baseServo);
        if (servoDone(&baseServo)) {
          currentStepState_Pet2 = StepState_Pet2::PetGrab2;
          timeCheckpoint = millis();
        }
        break;
        // --- End PetFound2 --- //

        // --- Begin PetGrab2 --- //
        case StepState_Pet2::PetGrab2:
        if (timeOfFlightReading < TOF_DETECTION_THRESHOLD || shoulderServo.targetAngle < 30) {
          delay(TOF_DETECTION_TIME_THRESHOLD);
          if (timeOfFlightReading < TOF_DETECTION_THRESHOLD || shoulderServo.targetAngle < 30) {
            servoGoTo(&clawServo, CLAW_CLOSED_ANGLE);
            delay(CLAW_CLOSE_TIME);
            currentStepState_Pet2 = StepState_Pet2::ReturnArm2;
            timeCheckpoint = millis();
          }
        }
        height = 12.0;
        nextShoulderAngle = shoulderServo.currentAngle - GRAB_SHOULDER_ANGLE_INCREMENT;
        nextElbowAngle = calculateElbowAngle(nextShoulderAngle, height);
        nextWristAngle = calculateWristAngle(nextShoulderAngle, nextElbowAngle, height);
        setAllServoTargets(baseServo.currentAngle, nextShoulderAngle, nextElbowAngle, nextWristAngle, clawServo.currentAngle);
        while (!allServosDone()) {
          updateServos();
          delay(20);
        }
        break;
        // --- End PetGrab2 --- //

        // --- Begin ReturnArm2 --- //
        case StepState_Pet2::ReturnArm2:
        setAllServoTargets(baseServo.currentAngle, 90, 160, 127, CLAW_CLOSED_ANGLE);
        while (!allServosDone()) {
          updateServos();
        } 
        if (allServosDone()) {
          currentStepState_Pet2 = StepState_Pet2::DropPet2;
          setServoTarget(&baseServo, 160);
          while (!servoDone(&baseServo)) {
            updateServo(&baseServo);
            delay(20);
          }
          timeCheckpoint = millis();
        }
        break; 
        // --- End ReturnArm2 --- //

        // --- Begin DropPet2 --- //
        case StepState_Pet2::DropPet2:
        setServoTarget(&shoulderServo, 25);
        setServoTarget(&elbowServo, 100);
        setServoTarget(&wristServo, 137);
        while (!servoDone(&shoulderServo) || !servoDone(&elbowServo) || !servoDone(&wristServo)) {
          updateServo(&shoulderServo);
          updateServo(&elbowServo);
          updateServo(&wristServo);
          delay(20);
        }
        servoGoTo(&clawServo, 130);
        delay(CLAW_CLOSE_TIME);
        servoGoTo(&clawServo, 20);
        setAllServoTargets(120, 70, 170, 160, 130);
        while (!allServosDone()) {
          updateServos();
          delay(20);
        }
        servoGoTo(&clawServo, 130);
        currentPetState = PetState::Pet3;
        setPIDValues(2.8, 0.5, 0.0, 0.0);
        resetError();
        timeCheckpoint = millis();
        break;
        // --- End DropPet2 --- //

      }
      break;
      // --- End Pet2 --- //

      // --- Begin Pet3 --- //
      case PetState::Pet3:
      switch (currentStepState_Pet3) {


        // --- Begin FindTarget3 --- //
        case StepState_Pet3::FindTarget3:
        runPID_withBackup(900);
        if (timeOfFlightReading < 120 && millis() - timeCheckpoint > PET3_DETECTION_TIMEOUT) { // was 90
          currentStepState_Pet3 = StepState_Pet3::ArmSearchPreset3;
          drivetrainBreak(1200);
          baseServo.speed = 0.02;
          timeCheckpoint = millis();
        }
        break;
        // --- End FindTarget3 --- //

        // --- Begin ArmSearchPreset3 --- //
        case StepState_Pet3::ArmSearchPreset3:
        setServoTarget(&baseServo, PET3_LOWER_SWEEP_ANGLE);
        updateServo(&baseServo);
        if (servoDone(&baseServo)) {
          currentStepState_Pet3 = StepState_Pet3::PetSearch3;
          baseServo.speed = 0.04;
          timeOfFlightSum = 0;
          timeOfFlightCount = 0;
          sweepAngleIncrement = 0;
          timeCheckpoint = millis();
        }
        break;
        // --- End ArmSearchPreset3 --- //

        // --- Begin PetSearch3 --- //
        case StepState_Pet3::PetSearch3:
        timeOfFlightSum += timeOfFlightReading;
        timeOfFlightCount++;
        if (millis() - timeCheckpoint > (int) ((double) PET3_SWEEP_TIME / (PET3_UPPER_SWEEP_ANGLE - PET3_LOWER_SWEEP_ANGLE))) {
          pet3TimeOfFlightArray[sweepAngleIncrement] = (double) timeOfFlightSum / timeOfFlightCount;
          timeOfFlightSum = 0;
          timeOfFlightCount = 0;
          sweepAngleIncrement++;
          servoGoTo(&baseServo, PET3_LOWER_SWEEP_ANGLE + sweepAngleIncrement);
          if (sweepAngleIncrement == PET3_UPPER_SWEEP_ANGLE - PET3_LOWER_SWEEP_ANGLE) {
            arrayValueMark = pet3TimeOfFlightArray[0];
            for (int i = 0; arrayValueMark > TOF_RANGE_THRESHOLD && i < PET3_UPPER_SWEEP_ANGLE - PET3_LOWER_SWEEP_ANGLE; i++) {
              if (pet3TimeOfFlightArray[i] < TOF_RANGE_THRESHOLD) {
                arrayValueMark = pet3TimeOfFlightArray[i];
                lowerAngleMark = i;
              }
            }
            arrayValueMark = pet3TimeOfFlightArray[PET3_UPPER_SWEEP_ANGLE - PET3_LOWER_SWEEP_ANGLE - 1];
            for (int i = PET3_UPPER_SWEEP_ANGLE - PET3_LOWER_SWEEP_ANGLE - 1; arrayValueMark > TOF_RANGE_THRESHOLD && i >= 0; i--) {
              if (pet3TimeOfFlightArray[i] < TOF_RANGE_THRESHOLD) {
                arrayValueMark = pet3TimeOfFlightArray[i];
                upperAngleMark = i;
              }
            }
            maxTOFbaseAngle = PET3_LOWER_SWEEP_ANGLE + (int) ((lowerAngleMark + upperAngleMark) / 2.0);
            currentStepState_Pet3 = StepState_Pet3::PetFound3;
          }
          timeCheckpoint = millis();
        }
        break;
        // --- End PetSearch3 --- //

        // --- Begin PetFound3 --- //
        case StepState_Pet3::PetFound3:
        baseServo.speed = SERVO_STARTING_SPEED;
        setServoTarget(&baseServo, maxTOFbaseAngle);
        while (!servoDone(&baseServo)) {
          updateServo(&baseServo);
          delay(20);
        }
        currentStepState_Pet3 = StepState_Pet3::PetGrab3;
        timeCheckpoint = millis();
        break;
        // --- End PetFound3 --- //

        // --- Begin PetGrab3 --- //
        case StepState_Pet3::PetGrab3:
        if (timeOfFlightReading < TOF_DETECTION_THRESHOLD || shoulderServo.targetAngle < 5) {
          delay(TOF_DETECTION_TIME_THRESHOLD);
          if (timeOfFlightReading < TOF_DETECTION_THRESHOLD || shoulderServo.targetAngle < 5) {
            servoGoTo(&clawServo, CLAW_CLOSED_ANGLE);
            delay(CLAW_CLOSE_TIME);
            currentStepState_Pet3 = StepState_Pet3::DropPet3;
            baseServo.speed = SERVO_STARTING_SPEED;
            timeCheckpoint = millis();
          }
        }
        nextElbowAngle = elbowServo.currentAngle - GRAB_SHOULDER_ANGLE_INCREMENT;
        nextWristAngle = wristServo.currentAngle - GRAB_SHOULDER_ANGLE_INCREMENT;
        setAllServoTargets(baseServo.currentAngle, shoulderServo.currentAngle, nextElbowAngle, nextWristAngle, clawServo.currentAngle);
        while (!allServosDone()) {
          updateServos();
        }
        break;
        // --- End PetGrab3 --- //

        // --- Begin DropPet3 --- //
        case StepState_Pet3::DropPet3:
        dropPetBasket();
        setAllServoTargets(160, 80, 140, 100, 130);
        while (!allServosDone()) {
          updateServos();
          delay(20);
        }
        currentPetState = PetState::Pet4;
        setPIDValues(2.1, 0.5, 0.0, 0.0);
        resetError();
        timeCheckpoint = millis();
        break;
        // --- End DropPet3 --- //

      }
      break;
      // --- End Pet3 --- //

      // --- Begin Pet4 --- //
      case PetState::Pet4:
      switch (currentStepState_Pet4) {

        // --- Begin FindTarget4 --- //
        case StepState_Pet4::FindTarget4:
        runPID_withBackup(900);
        if (timeOfFlightReading < 240 && millis() - timeCheckpoint > PET4_DETECTION_TIMEOUT) {
          currentStepState_Pet4 = StepState_Pet4::ArmSearchPreset4;
          drivetrainBreak(900);
          timeCheckpoint = millis();
        }
        break;
        // --- End FindTarget4 --- //

        // --- Begin ArmSearchPreset4 --- //
        case StepState_Pet4::ArmSearchPreset4:
        setAllServoTargets(130, 50, 140, 120, PET4_LOWER_SWEEP_ANGLE);
        updateServos();
        if (allServosDone()) {
          currentStepState_Pet4 = StepState_Pet4::PetSearch4;
          baseServo.speed = 0.02;
          timeOfFlightSum = 0;
          timeOfFlightCount = 0;
          sweepAngleIncrement = 0;
          timeCheckpoint = millis();

          // ARTIFICIAL
          baseServo.speed = SERVO_STARTING_SPEED;
          setAllServoTargets(150, 50, 140, 115, 120);
          while (!allServosDone()) {
            updateServos();
          }
          currentPetState = PetState::Pet5;
          setPIDValues(2.1, 0.5, 4.0, 0.0);
          resetError();
          timeCheckpoint = millis();
          }
          break;
        // --- End ArmSearchPreset4 --- //

        // --- Begin PetSearch4 --- //
        case StepState_Pet4::PetSearch4:
        timeOfFlightSum += timeOfFlightReading;
        timeOfFlightCount++;
        if (millis() - timeCheckpoint > (int) ((double) PET4_SWEEP_TIME / (PET4_UPPER_SWEEP_ANGLE - PET4_LOWER_SWEEP_ANGLE))) {
          pet4TimeOfFlightArray[sweepAngleIncrement] = (double) timeOfFlightSum / timeOfFlightCount;
          timeOfFlightSum = 0;
          timeOfFlightCount = 0;
          sweepAngleIncrement++;
          servoGoTo(&baseServo, PET4_LOWER_SWEEP_ANGLE + sweepAngleIncrement);
          if (sweepAngleIncrement == PET4_UPPER_SWEEP_ANGLE - PET4_LOWER_SWEEP_ANGLE) {
            arrayValueMark = pet4TimeOfFlightArray[0];
            for (int i = 0; arrayValueMark > TOF_RANGE_THRESHOLD && i < PET4_UPPER_SWEEP_ANGLE - PET4_LOWER_SWEEP_ANGLE; i++) {
              if (pet4TimeOfFlightArray[i] < TOF_RANGE_THRESHOLD) {
                arrayValueMark = pet4TimeOfFlightArray[i];
                lowerAngleMark = i;
              }
            }
            arrayValueMark = pet4TimeOfFlightArray[PET4_UPPER_SWEEP_ANGLE - PET4_LOWER_SWEEP_ANGLE - 1];
            for (int i = PET4_UPPER_SWEEP_ANGLE - PET4_LOWER_SWEEP_ANGLE - 1; arrayValueMark > TOF_RANGE_THRESHOLD && i >= 0; i--) {
              if (pet4TimeOfFlightArray[i] < TOF_RANGE_THRESHOLD) {
                arrayValueMark = pet4TimeOfFlightArray[i];
                upperAngleMark = i;
              }
            }
            maxTOFbaseAngle = PET4_LOWER_SWEEP_ANGLE + (int) ((lowerAngleMark + upperAngleMark) / 2.0);
            currentStepState_Pet4 = StepState_Pet4::PetFound4;
          }
          timeCheckpoint = millis();
        }
        break;
        // --- End PetSearch4 --- //

        // --- Begin PetFound4 --- //
        case StepState_Pet4::PetFound4:
        // servoGoTo(&clawServo, 50);
        setServoTarget(&baseServo, maxTOFbaseAngle);
        while (!servoDone(&baseServo)) {
          updateServo(&baseServo);
          delay(20);
        } 
        currentStepState_Pet4 = StepState_Pet4::PetGrab4;
        timeCheckpoint = millis();
        break;
        // --- End PetFound4 --- //

        // --- Begin PetGrab4 --- //
        case StepState_Pet4::PetGrab4:
        // dislodgePet();
        currentStepState_Pet4 = StepState_Pet4::DropPet4;
        timeCheckpoint = millis();
        break;
        // --- End PetGrab4 --- //

        // --- Begin DropPet4 --- //
        case StepState_Pet4::DropPet4:
        setServoTarget(&wristServo, 180);
        while(!servoDone(&wristServo)) {
          updateServo(&wristServo);
        }
        dropPetBasket();
        baseServo.speed = SERVO_STARTING_SPEED;
        setAllServoTargets(150, 50, 140, 115, 120);
        while (!allServosDone()) {
          updateServos();
        }
        currentPetState = PetState::Pet5;
        setPIDValues(2.1, 0.5, 4.0, 0.0);
        resetError();
        timeCheckpoint = millis();
        break;  
        // --- End DropPet4 --- //

      }   
      break;
      // --- End Pet4 --- //

      // --- Begin Pet5 --- //
      case PetState::Pet5:
      switch (currentStepState_Pet5) {

        // --- Begin FindTarget5 --- //
        case StepState_Pet5::FindTarget5:
        runPID_withHysteresis(800);
        if (timeOfFlightReading < 240 && timeOfFlightReading > 40 && millis() - timeCheckpoint > PET5_DETECTION_TIMEOUT) {
          currentStepState_Pet5 = StepState_Pet5::ArmSearchPreset5;
          drivetrainBreak(800);
          timeCheckpoint = millis();
        }
        break;
        // --- End FindTarget5 --- //

        // --- Begin ArmSearchPreset5 --- //
        case StepState_Pet5::ArmSearchPreset5:
        setServoTarget(&baseServo, 130);
        updateServo(&baseServo);
        if (servoDone(&baseServo)) {
          currentStepState_Pet5 = StepState_Pet5::PetSearch5;
          baseServo.speed = 0.04;
          timeOfFlightSum = 0;
          timeOfFlightCount = 0;
          sweepAngleIncrement = 0;
          timeCheckpoint = millis();
        }
        break;
        // --- End ArmSearchPreset5 --- //

        // --- Begin PetSearch5 --- //
        case StepState_Pet5::PetSearch5:
        timeOfFlightSum += timeOfFlightReading;
        timeOfFlightCount++;
        if (millis() - timeCheckpoint > (int) ((double) PET5_SWEEP_TIME / (PET5_UPPER_SWEEP_ANGLE - PET5_LOWER_SWEEP_ANGLE))) {
          pet5TimeOfFlightArray[sweepAngleIncrement] = (double) timeOfFlightSum / timeOfFlightCount;
          timeOfFlightSum = 0;
          timeOfFlightCount = 0;
          sweepAngleIncrement++;
          servoGoTo(&baseServo, PET5_LOWER_SWEEP_ANGLE + sweepAngleIncrement);
          if (sweepAngleIncrement == PET5_UPPER_SWEEP_ANGLE - PET5_LOWER_SWEEP_ANGLE) {
            arrayValueMark = pet5TimeOfFlightArray[0];
            for (int i = 0; arrayValueMark > TOF_RANGE_THRESHOLD && i < PET5_UPPER_SWEEP_ANGLE - PET5_LOWER_SWEEP_ANGLE; i++) {
              if (pet5TimeOfFlightArray[i] < TOF_RANGE_THRESHOLD) {
                arrayValueMark = pet5TimeOfFlightArray[i];
                lowerAngleMark = i;
              }
            }
            arrayValueMark = pet5TimeOfFlightArray[PET5_UPPER_SWEEP_ANGLE - PET5_LOWER_SWEEP_ANGLE - 1];
            for (int i = PET5_UPPER_SWEEP_ANGLE - PET5_LOWER_SWEEP_ANGLE - 1; arrayValueMark > TOF_RANGE_THRESHOLD && i >= 0; i--) {
              if (pet5TimeOfFlightArray[i] < TOF_RANGE_THRESHOLD) {
                arrayValueMark = pet5TimeOfFlightArray[i];
                upperAngleMark = i;
              }
            }
            maxTOFbaseAngle = PET5_LOWER_SWEEP_ANGLE + (int) ((lowerAngleMark + upperAngleMark) / 2.0);
            currentStepState_Pet5 = StepState_Pet5::PetFound5;
          }
          timeCheckpoint = millis();
        }
        break;
        // --- End PetSearch5 --- //

        // --- Begin PetFound5 --- //
        case StepState_Pet5::PetFound5:
        setServoTarget(&baseServo, maxTOFbaseAngle);
        while (!servoDone(&baseServo)) {
          updateServo(&baseServo);
          delay(20);
        } 
        currentStepState_Pet5 = StepState_Pet5::PetGrab5;
        timeCheckpoint = millis();
        break;
        // --- End PetFound5 --- //

        // --- Begin PetGrab5 --- //
        case StepState_Pet5::PetGrab5:
        if (timeOfFlightReading < TOF_DETECTION_THRESHOLD || shoulderServo.targetAngle < 5) {
          delay(TOF_DETECTION_TIME_THRESHOLD);
          if (timeOfFlightReading < TOF_DETECTION_THRESHOLD || shoulderServo.targetAngle < 5) {
            servoGoTo(&clawServo, CLAW_CLOSED_ANGLE);
            delay(CLAW_CLOSE_TIME);
            currentStepState_Pet5 = StepState_Pet5::DropPet5;
            timeCheckpoint = millis();
          }
        }
        height = 12.0;
        nextShoulderAngle = shoulderServo.currentAngle - GRAB_SHOULDER_ANGLE_INCREMENT;
        nextElbowAngle = calculateElbowAngle_HIGHER(nextShoulderAngle, height);
        nextWristAngle = calculateWristAngle_HIGHER(nextShoulderAngle, nextElbowAngle, height);
        setAllServoTargets(baseServo.currentAngle, nextShoulderAngle, nextElbowAngle, nextWristAngle, clawServo.currentAngle);
        while (!allServosDone()) {
          updateServos();
          delay(20);
        }
        break;
        // --- End PetGrab5 --- //

        // --- Begin DropPet5 --- //
        case StepState_Pet5::DropPet5:
        baseServo.speed = SERVO_STARTING_SPEED;
        setServoTarget(&wristServo, 180);
        while(!servoDone(&wristServo)) {
          updateServo(&wristServo);
          delay(20);
        }
        setServoTarget(&baseServo, 90);
        setServoTarget(&shoulderServo, 70);
        setServoTarget(&elbowServo, 130);
        while(!servoDone(&baseServo) || !servoDone(&shoulderServo) || !servoDone(&elbowServo)) {
          updateServo(&baseServo);
          updateServo(&shoulderServo);
          updateServo(&elbowServo);
          delay(20);
        }
        dropPetBasket();
        setAllServoTargets(10, 40, 130, 130, 120);
        while (!allServosDone()) {
          updateServos();
          delay(20);
        }
        currentPetState = PetState::Pet6;
        setPIDValues(1.2, 0.0, 0.0, 0.0);
        resetError();  
        timeCheckpoint = millis();
        break;
        // --- End DropPet5 --- //

      }
      break; 
      // --- End Pet5 --- // 

      // --- Begin Pet6 --- //
      case PetState::Pet6:
      switch (currentStepState_Pet6) {

        // --- Begin FindTarget6 --- //
        case StepState_Pet6::FindTarget6:
        runPID_withBackup(700);
        if (timeOfFlightReading < 110) {
          currentStepState_Pet6 = StepState_Pet6::ArmSearchPreset6;
          drivetrainBreak(700);
          timeCheckpoint = millis();
        } 
        break;
        // --- End FindTarget6 --- //

        // --- Begin ArmSearchPreset6 --- //
        case StepState_Pet6::ArmSearchPreset6:
        setServoTarget(&baseServo, 50);
        updateServo(&baseServo);
        if (servoDone(&baseServo)) {
          currentStepState_Pet6 = StepState_Pet6::PetSearch6;
          maxMagnetometerReading = 0.0;
          maxMagnetometerBaseAngle = baseServo.currentAngle;
          baseServo.speed = 0.04;
          delay(500);
          timeCheckpoint = millis();
        }
        break;
        // --- End ArmSearchPreset6 --- //

        // --- Begin PetSearch6 --- //
        case StepState_Pet6::PetSearch6:
        setServoTarget(&baseServo, 10);
        updateServo(&baseServo);
        readMagnetometer();
        if (magnetometerMagnitude > maxMagnetometerReading) {
          maxMagnetometerReading = magnetometerMagnitude;
          maxMagnetometerBaseAngle = baseServo.currentAngle;
        }
        if (servoDone(&baseServo)) {
          currentStepState_Pet6 = StepState_Pet6::PetFound6;
          baseServo.speed = SERVO_STARTING_SPEED;
          timeCheckpoint = millis();
        }
        break;
        // --- End PetSearch6 --- //

        // --- Begin PetFound6 --- //
        case StepState_Pet6::PetFound6:
        setServoTarget(&baseServo, maxMagnetometerBaseAngle);
        while (!servoDone(&baseServo)) {
          updateServo(&baseServo);
          delay(20);
        } 
        currentStepState_Pet6 = StepState_Pet6::PetGrab6;
        timeCheckpoint = millis();
        break;
        // --- End PetFound6 --- //

        // --- Begin PetGrab6 --- //
        case StepState_Pet6::PetGrab6:
        if (timeOfFlightReading < TOF_DETECTION_THRESHOLD - 5 || shoulderServo.targetAngle < 10) {
          delay(TOF_DETECTION_TIME_THRESHOLD);
          if (timeOfFlightReading < TOF_DETECTION_THRESHOLD - 5 || shoulderServo.targetAngle < 10) {
            servoGoTo(&clawServo, CLAW_CLOSED_ANGLE);
            delay(CLAW_CLOSE_TIME);
            currentStepState_Pet6 = StepState_Pet6::DropPet6;
            timeCheckpoint = millis();
          }
        }
        height = 12.0;
        nextShoulderAngle = shoulderServo.currentAngle - GRAB_SHOULDER_ANGLE_INCREMENT;
        nextElbowAngle = 10.0 + calculateElbowAngle(nextShoulderAngle, height);
        nextWristAngle = -10.0 + calculateWristAngle(nextShoulderAngle, nextElbowAngle, height);
        setAllServoTargets(baseServo.currentAngle, nextShoulderAngle, nextElbowAngle, nextWristAngle, clawServo.currentAngle);
        while (!allServosDone()) {
          updateServos();
          delay(20);
        }
        break;
        // --- End PetGrab6 --- //

        // --- Begin DropPet6 --- //
        case StepState_Pet6::DropPet6:
        setServoTarget(&wristServo, 180);
        while(!servoDone(&wristServo)) {
          updateServo(&wristServo);
          delay(20);
        }
        setServoTarget(&baseServo, 90);
        setServoTarget(&shoulderServo, 70);
        setServoTarget(&elbowServo, 130);
        while(!servoDone(&baseServo) || !servoDone(&shoulderServo) || !servoDone(&elbowServo)) {
          updateServo(&baseServo);
          updateServo(&shoulderServo);
          updateServo(&elbowServo);
          delay(20);
        }
        
        setServoTarget(&shoulderServo, 70);
        setServoTarget(&baseServo, 90);
        setServoTarget(&wristServo, 180);
        while(!servoDone(&baseServo) || !servoDone(&baseServo) || !servoDone(&shoulderServo)) {
          updateServo(&baseServo);
          updateServo(&shoulderServo);
          updateServo(&wristServo);
          delay(20);
        }
        setServoTarget(&elbowServo, 30);
        setServoTarget(&shoulderServo, 80);
        while(!servoDone(&elbowServo) || !servoDone(&shoulderServo)) {
          updateServo(&elbowServo);
          updateServo(&shoulderServo);
          delay(20);
        }
        servoGoTo(&clawServo, 130);
        delay(CLAW_CLOSE_TIME);
        setServoTarget(&elbowServo, 150);
        while(!servoDone(&elbowServo)) {
          updateServo(&elbowServo);
          delay(20);
        }
        servoGoTo(&clawServo, 130);

        currentPetState = PetState::Debris;
        setPIDValues(2.1, 0.5, 4.0, 0.0);
        resetError();  
        timeCheckpoint = millis();
        break;
        // --- End DropPet6 --- //

      }
      break;
      // --- End Pet6 --- //

      // --- Begin Debris --- //
      case PetState::Debris:
      switch (currentStepState_Debris) {

        // --- Begin FindDebris --- //
        case StepState_Debris::FindDebris:
        runPID_withHysteresis(900);
        if (millis() - timeCheckpoint > 2000) {
          currentStepState_Debris = StepState_Debris::Cross;
          drivetrainSetPower(0);
          timeCheckpoint = millis();
        }
        break;
        // --- End FindDebris --- //

        // --- Begin Cross --- //
        case StepState_Debris::Cross:
        drivetrainSetPower(3000);
        if (millis() - timeCheckpoint > DEBRIS_CROSS_TIME) {
          currentStepState_Debris = StepState_Debris::Backup;
          drivetrainSetPower(0);
          timeCheckpoint = millis();
        }
        break;
        // --- End Cross --- //

        // --- Begin Backup --- //
        case StepState_Debris::Backup:
        drivetrainSetPower(-700);
        if (millis() - timeCheckpoint > DEBRIS_BACKUP_TIME) {
          currentStepState_Debris = StepState_Debris::FindTape;
          drivetrainSetPower(0);
          timeCheckpoint = millis();
        }
        break;
        // --- End Backup --- //

        // --- Begin FindTape --- //
        case StepState_Debris::FindTape:
        readReflectanceSensors();
        if (millis() - timeCheckpoint < DEBRIS_SWEEP_RIGHT_TIME) {
          rightMotorSetPower(1500);
        } else {
          leftMotorSetPower(1500);
        }
        if (leftOnTape && rightOnTape) {
          currentPetState = PetState::Pet7;
          drivetrainSetPower(0);
          setAllServoTargets(170, 70, 90, 0, 130);
          while (!allServosDone()) {
            updateServos();
            delay(20);
          }
          timeCheckpoint = millis();
        }
        break;
        // --- End FindTape --- //

      }
      break;
      // --- End Debris --- //

      // --- Begin Pet7 --- //
      case PetState::Pet7:
      switch (currentStepState_Pet7) {

        // --- Begin FindTarget7 --- //
        case StepState_Pet7::FindTarget7:
        runPID_withBackup(700);
        if (timeOfFlightReading < 90) {
          currentStepState_Pet7 = StepState_Pet7::ArmSearchPreset7;
          drivetrainBreak(700);
          timeCheckpoint = millis();
          if (timeOfFlightReading < TOF_NO_SWEEP_THRESHOLD) {
            currentStepState_Pet7 = StepState_Pet7::PetGrab7;
          }
        } 
        break;
        // --- End FindTarget7 --- //

        // --- Begin ArmSearchPreset7 --- //
        case StepState_Pet7::ArmSearchPreset7:
        setServoTarget(&baseServo, 180);
        updateServo(&baseServo);
        if (servoDone(&baseServo)) {
          currentStepState_Pet7 = StepState_Pet7::PetSearch7;
          maxMagnetometerReading = 0.0;
          maxMagnetometerBaseAngle = baseServo.currentAngle;
          baseServo.speed = 0.04;
          delay(500);
          timeCheckpoint = millis();
        }
        break;
        // --- End ArmSearchPreset7 --- //

        // --- Begin PetSearch7 --- //
        case StepState_Pet7::PetSearch7:
        setServoTarget(&baseServo, 130);
        updateServo(&baseServo);
        readMagnetometer();
        if (magnetometerMagnitude > maxMagnetometerReading) {
          maxMagnetometerReading = magnetometerMagnitude;
          maxMagnetometerBaseAngle = baseServo.currentAngle;
        }
        if (servoDone(&baseServo)) {
          currentStepState_Pet7 = StepState_Pet7::PetFound7;
          baseServo.speed = SERVO_STARTING_SPEED;
          timeCheckpoint = millis();
        }
        break;
        // --- End PetSearch7 --- //

        // --- Begin PetFound7 --- //
        case StepState_Pet7::PetFound7:
        setServoTarget(&baseServo, maxMagnetometerBaseAngle);
        while (!servoDone(&baseServo)) {
          updateServo(&baseServo);
          delay(20);
        } 
        currentStepState_Pet7 = StepState_Pet7::PetGrab7;
        timeCheckpoint = millis();
        break;
        // --- End PetFound7 --- //

        // --- Begin PetGrab7 --- //
        case StepState_Pet7::PetGrab7:
        if (timeOfFlightReading < TOF_DETECTION_THRESHOLD || shoulderServo.targetAngle < 5) {
          delay(TOF_DETECTION_TIME_THRESHOLD);
          if (timeOfFlightReading < TOF_DETECTION_THRESHOLD || shoulderServo.targetAngle < 5) {
            servoGoTo(&clawServo, CLAW_CLOSED_ANGLE);
            delay(CLAW_CLOSE_TIME);
            currentStepState_Pet7 = StepState_Pet7::DropPet7;
            timeCheckpoint = millis();
          }
        }
        height = 18.0;
        nextShoulderAngle = shoulderServo.currentAngle - GRAB_SHOULDER_ANGLE_INCREMENT;
        nextElbowAngle = calculateElbowAngle(nextShoulderAngle, height);
        nextWristAngle = -5.0 + calculateWristAngle(nextShoulderAngle, nextElbowAngle, height);
        setAllServoTargets(baseServo.currentAngle, nextShoulderAngle, nextElbowAngle, nextWristAngle, clawServo.currentAngle);
        while (!allServosDone()) {
          updateServos();
          delay(20);
        }
        break;
        // --- End PetGrab7 --- //

        // --- Begin PetDrop7 --- //
        case StepState_Pet7::DropPet7:
        setServoTarget(&wristServo, 140);
        setServoTarget(&shoulderServo, 40);
        while (!servoDone(&wristServo) || !servoDone(&shoulderServo)) {
          updateServo(&wristServo);
          updateServo(&shoulderServo);
          delay(20);
        }
        setServoTarget(&baseServo, 90);
        while (!servoDone(&baseServo)) {
          updateServo(&baseServo);
          delay(20);
        }
        setAllServoTargets(90, 90, 160, 142, CLAW_CLOSED_ANGLE);
        while (!allServosDone()) {
          updateServos();
          delay(20);
        }
        dropPetBasket();
        setServoTarget(&shoulderServo, 60);
        while (!servoDone(&shoulderServo)) {
          updateServo(&shoulderServo);
          delay(20);
        }
        currentPetState = PetState::PostPet;
        setPIDValues(1.2, 0.0, 0.0, 0.0);
        timeCheckpoint = millis();
        break;
        // --- End PetDrop7 --- //

      }
      break;
      // --- End Pet7 --- //

      // --- Begin PostPet --- //
      case PetState::PostPet:
      switch (currentStepState_PostPet) {

        // --- Begin FindEdge --- //
        case StepState_PostPet::FindEdge:
        drivetrainSetPower(900);
        if (millis() - timeCheckpoint > CLEAR_ZIPLINE_TIME) {
          currentStepState_PostPet = StepState_PostPet::RaiseBasket;
          drivetrainBreak(900);
          baseServo.speed = SERVO_STARTING_SPEED;
          timeCheckpoint = millis();
        }
        break;
        // --- End FindEdge --- //

        // --- Begin RaiseBasket --- //
        case StepState_PostPet::RaiseBasket:
        verticalMotorSetPower(3600);
        setAllServoTargets(10, 10, 100, 150, 120);
        updateServos();
        if (millis() - timeCheckpoint > RAISE_BASKET_TIME) {
          currentStepState_PostPet = StepState_PostPet::Align;
          verticalMotorSetPower(0);
          timeCheckpoint = millis();
        }
        break;
        // --- End RaiseBasket --- //

        // --- Begin Align --- //
        case StepState_PostPet::Align:
        leftMotorSetPower(1000);
        rightMotorSetPower(-1000);
        if (millis() - timeCheckpoint > 500) {
          currentStepState_PostPet = StepState_PostPet::Reverse;
          timeCheckpoint = millis();
        }
        break;
        // --- End Align --- //

        // --- Begin Reverse --- //
        case StepState_PostPet::Reverse:
        if (millis() - timeCheckpoint < 5000) {
          drivetrainSetPower(-1000);
        }
        if (millis() - timeCheckpoint > 3000) {
          verticalMotorSetPower(-3000);
        }
        break;
        // --- End Reverse --- //

      }
      break;
      // --- End PostPet --- //

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
void dropPetBasket() {
  setServoTarget(&shoulderServo, 70);
  setServoTarget(&baseServo, 90);
  setServoTarget(&wristServo, 180);
  while(!servoDone(&baseServo) || !servoDone(&baseServo) || !servoDone(&shoulderServo)) {
    updateServo(&baseServo);
    updateServo(&shoulderServo);
    updateServo(&wristServo);
    delay(20);
  }
  setServoTarget(&elbowServo, 30);
  setServoTarget(&shoulderServo, 80);
  while(!servoDone(&elbowServo) || !servoDone(&shoulderServo)) {
    updateServo(&elbowServo);
    updateServo(&shoulderServo);
    delay(20);
  }
  delay(300);
  servoGoTo(&clawServo, 130);
  delay(CLAW_CLOSE_TIME);
  servoGoTo(&clawServo, 60);
  delay(1000);
  setServoTarget(&elbowServo, 150);
  while(!servoDone(&elbowServo)) {
    updateServo(&elbowServo);
    delay(20);
  }
  servoGoTo(&clawServo, 130);
}

void dislodgePet() {
  setServoTarget(&wristServo, 165);
  while(!servoDone(&wristServo)) {
    updateServo(&wristServo);
    delay(20);
  }
  servoGoTo(&clawServo, 35);
  delay(500);
  setServoTarget(&elbowServo, 90);
  setServoTarget(&wristServo, 110);
  while(!servoDone(&elbowServo) || !servoDone(&wristServo)) {
    updateServo(&elbowServo);
    updateServo(&wristServo);
    if (timeOfFlightReading < TOF_DETECTION_THRESHOLD) {
      servoGoTo(&clawServo, CLAW_CLOSED_ANGLE);
      return;
    }
  }
  servoGoTo(&clawServo, CLAW_CLOSED_ANGLE);
}

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
  readReflectanceSensors();
  if (abs(leftReflectance - rightReflectance) < REFLECTANCE_COMPARISON_THRESHOLD) {
    servoGoTo(&clawServo, 60);
  } else {
    servoGoTo(&clawServo, SERVO_STARTING_ANGLES[4]);
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
      
      if (presetCalibration) {
        leftReflectanceThreshold = REFLECTANCE_PRESET_CALIBRATION;
        rightReflectanceThreshold = REFLECTANCE_PRESET_CALIBRATION;
      }

      servoGoTo(&clawServo, SERVO_STARTING_ANGLES[NUM_SERVOS - 1]);
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

void runPID_onRamp(int power) {
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
    drivetrainSetPower(-500);
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

void resetError() {
  readReflectanceSensors();
  computePID();
}

void drivetrainBreak(int power) {
  breakTime = millis();
  while (millis() - breakTime < 100) {
    drivetrainSetPower(-power);
  }
  drivetrainSetPower(0);
}

void runPID_withCorrection(int power) {
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
    if (prevLeftOnTape) {
      rightMotorSetPower((int) (power / 2.0));
      leftMotorSetPower((int) (-power / 2.0));
    } else if (prevRightOnTape) {
      rightMotorSetPower((int) (-power / 2.0));
      leftMotorSetPower((int) (power / 2.0));
    }
    if (leftOnTape && rightOnTape) {
      computePID();
      currentTaskState = TaskState::TapeFollow;
    }
    break; 
  }
}
