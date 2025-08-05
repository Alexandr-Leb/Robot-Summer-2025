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
const int REFLECTANCE_COMPARISON_THRESHOLD = 150;
const int NUM_REFLECTANCE_AVERAGE_COUNT = 10;
const int NUM_REFLECTANCE_AVERAGE_COUNT_TAPE = 5;
const int FORWARD_REFLECTANCE_COMPARISON_THRESHOLD = 500;
const int TOF_DETECTION_THRESHOLD = 35;
const int TOF_RANGE_THRESHOLD = 240;

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
Adafruit_VL6180X tof = Adafruit_VL6180X();
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

// --- Runtime Parameters --- //
// Runtime Parameters - General
const double CLAW_CLOSED_ANGLE = 2.0;
const int CLAW_CLOSE_TIME = 1500;

// Runtime Parameters - PrePet
const int BUCKET_CLEAR_TIME = 2000;
const int GATE_CLEAR_TIME = 800;

// Runtime Parameters - Pet1
const int LIFT_BASKET_TIME = 2500;
const int PET1_GRAB_TIME = 5000;

// Runtime Parameters - Ramp
const int RAMP_DETECTION_THRESHOLD = 300;
const int RAMP_CORREECTION_TIMEOUT = 500;
const int CLIMB_TIME_BEFOE_DROP = 1000;
const int INCH_FORWARDS_TIME = 200;
 
// Runtime Parameters - Pet3
const int PET3_LOWER_SWEEP_ANGLE = 10;
const int PET3_UPPER_SWEEP_ANGLE = 90;
const int PET3_SWEEP_TIME = 3000;
double timeOfFlightArray[PET3_UPPER_SWEEP_ANGLE - PET3_LOWER_SWEEP_ANGLE];

// --- Function Headers --- //
// Function Headers - Runtime Functions

// Function Headers - Switch State Functions
void initializeState();
void resetState();
void offState();
void updateSwitchState();

// Function Headers - PID Functions
void runPID_withBackup(int power);
void runPID_withHysteresis(int power);
void setPIDValues(double p, double d, double hysteresis, double e);
void pidSetup();
void computePID();
void runHysteresis(int power);

void setup() {
  drivetrainSetup();
  sensorSetup();
  armSetup();
  pidSetup();
  stateSetup();

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

  currentTaskState = TaskState::TapeFollow;
  setPIDValues(2.1, 0.5, 0.0, 0.0);

  Serial.begin(115200);
  
  // Artificial Start Pet3
  // delay(1000);
  // currentPetState = Pet3;
  // setAllServoTargets(120, 70, 170, 160, 130);
  // while (!allServosDone()) {
  //   updateServos();
  // }
  // currentPetState = PetState::Pet3;
  // setPIDValues(1.2, 0.0, 0.0, 0.0);
  // delay(1000);
  // timeCheckpoint = millis();
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
          setPIDValues(1.2, 0.0, 0.0, 0.0);
          timeCheckpoint = millis();
        }
        break;
        // --- End clearBucket --- //

        // --- Begin FindGate --- //
        case StepState_PrePet::FindGate:
        runPID_withBackup(700);
        if (timeOfFlightReading < 60) {
          currentStepState_PrePet = StepState_PrePet::ClearDoorway;
          setPIDValues(1.2, 0.0, 5.0, 0.0);
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
        readReflectanceSensors();
        computePID();
        setAllServoTargets(65, 50, 177, 142, 120);
        while (!allServosDone()) {
          updateServos();
        }
        setAllServoTargets(65, 70, 177, 142, 120);
        while (!allServosDone()) {
          updateServos();
        }
        if (allServosDone()) {
          currentPetState = PetState::Pet1;
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
        if (timeOfFlightReading < 120) { 
          currentStepState_Pet1 = StepState_Pet1::LiftBasket;
          drivetrainSetPower(0);
          timeCheckpoint = millis();
        }
        break;
        // --- End FindTarget1 --- //

        // --- Begin LiftBasket --- //
        case StepState_Pet1::LiftBasket:
        verticalMotorSetPower(2000); 
        readReflectanceSensors();
        computePID();
        if (millis() - timeCheckpoint > LIFT_BASKET_TIME) {
          currentStepState_Pet1 = StepState_Pet1::ArmSearchPreset1;
          if (timeOfFlightReading < 30) {
            currentStepState_Pet1 = StepState_Pet1::PetGrab1;
          }
          verticalMotorSetPower(0);
          timeCheckpoint = millis();
        }
        break;
        // --- End LiftBasket --- //

        // --- Begin ArmSearchPreset1 --- //
        case StepState_Pet1::ArmSearchPreset1:
        setServoTarget(&baseServo, 90);
        updateServo(&baseServo);
        if (servoDone(&baseServo)) {
          currentStepState_Pet1 = StepState_Pet1::PetSearch1;
          maxMagnetometerReading = 0.0;
          maxMagnetometerBaseAngle = baseServo.currentAngle;
          baseServo.speed = 0.02;
          timeCheckpoint = millis();
        }
        break;
        // --- End ArmSearchPreset1 --- //

        // --- Begin PetSearch1 --- //
        case StepState_Pet1::PetSearch1:
        setServoTarget(&baseServo, 50);
        updateServo(&baseServo);
        readMagnetometer();
        if (magnetometerMagnitude > maxMagnetometerReading) {
          maxMagnetometerReading = magnetometerMagnitude;
          maxMagnetometerBaseAngle = baseServo.currentAngle;
        }
        if (servoDone(&baseServo)) {
          currentStepState_Pet1 = StepState_Pet1::PetFound1;
          baseServo.speed = SERVO_STARTING_SPEED;
          timeCheckpoint = millis();
        }
        break;
        // --- End PetSearch1 --- //

        // --- Begin PetFound1 --- //
        case StepState_Pet1::PetFound1:
        setServoTarget(&baseServo, maxMagnetometerBaseAngle);
        updateServo(&baseServo);
        if (servoDone(&baseServo)) {
          currentStepState_Pet1 = StepState_Pet1::PetGrab1;
          timeCheckpoint = millis();
        }
        break;
        // --- End PetFound1 --- //

        // --- Begin PetGrab1 --- // 
        case StepState_Pet1::PetGrab1:
        height = 12.0;
        nextShoulderAngle = shoulderServo.currentAngle - 1;
        nextElbowAngle = calculateElbowAngle(nextShoulderAngle, height);
        nextWristAngle = calculateWristAngle(nextShoulderAngle, nextElbowAngle, height);
        setAllServoTargets(baseServo.currentAngle, nextShoulderAngle, nextElbowAngle, nextWristAngle, clawServo.currentAngle);
        while (!allServosDone()) {
          updateServos();
          delay(20);
        }
        if (timeOfFlightReading < TOF_DETECTION_THRESHOLD || shoulderServo.targetAngle < 5) {
          servoGoTo(&clawServo, CLAW_CLOSED_ANGLE);
          currentStepState_Pet1 = StepState_Pet1::ReturnArm1;
          delay(CLAW_CLOSE_TIME);
          timeCheckpoint = millis();  
        }
        break;
        // --- End PetGrab1 --- //

        // --- Begin ReturnArm1 --- //
        case StepState_Pet1::ReturnArm1:
        setAllServoTargets(90, 90, 160, 142, CLAW_CLOSED_ANGLE);
        while (!allServosDone()) {
          updateServos();
        }
        if (allServosDone()) {
          currentPetState = PetState::Ramp;
          setPIDValues(1.2, 0.0, 0.0, 0.0);
          forwardLeftReflectanceSum = 0;
          forwardReflectanceCount = 0;
          timeCheckpoint = millis();
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
        runPID_withBackup(700);
        readForwardReflectanceSensors();
        forwardLeftReflectanceSum += forwardLeftReflectance;
        forwardReflectanceCount++;
        if (forwardReflectanceCount == NUM_REFLECTANCE_AVERAGE_COUNT - 1) {
          if (forwardLeftReflectanceSum / forwardReflectanceCount < RAMP_DETECTION_THRESHOLD) {
            currentStepState_Ramp = StepState_Ramp::AlignRamp;
            timeCheckpoint = millis();
            while (millis() - timeCheckpoint < 800) {
              leftMotorSetPower(1500);
              rightMotorSetPower(700);
            }
            drivetrainSetPower(0);
            leftReflectanceSum = 0;
            reflectanceCount = 0;
            timeCheckpoint = millis();
          }
          forwardLeftReflectanceSum = 0;
          forwardReflectanceCount = 0;
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
            timeCheckpoint = millis();
            while (millis() - timeCheckpoint < CLIMB_TIME_BEFOE_DROP) {
              runPID_withHysteresis(1600);
            }
            currentStepState_Ramp = StepState_Ramp::DropPet1;
            setPIDValues(1.8, 0.0, 4.0, 0.0);
            readReflectanceSensors();
            readReflectanceSensors();
            timeCheckpoint = millis();
          }
          leftReflectanceSum = 0;
          reflectanceCount = 0;
        }
        break;
        // --- End AlignRamp --- //

        // --- Begin DropPet1 --- //
        case StepState_Ramp::DropPet1:
        runPID_withHysteresis(900);
        if (millis() - timeCheckpoint <= 1000) {
          setAllServoTargets(180, 60, 110, 120, CLAW_CLOSED_ANGLE);
          updateServos();
        } else if (millis() - timeCheckpoint > 1000 && millis() - timeCheckpoint <= 1800) {
          servoGoTo(&clawServo, 120);
        } else if (millis() - timeCheckpoint > 1800 && millis() - timeCheckpoint <= 2600) {
          setAllServoTargets(90, 70, 177, 142, 120);
          updateServos();
        } else {
          currentStepState_Ramp = StepState_Ramp::ClimbRamp;
          forwardLeftReflectanceSum = 0;
          forwardRightReflectanceSum = 0;
          forwardReflectanceCount = 0;
          timeCheckpoint = millis();
        }
        break;
        // --- End DropPet1 --- //

        // --- Begin ClimbRamp --- //
        case StepState_Ramp::ClimbRamp:
        runPID_withHysteresis(1400);
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
        break;
        // --- End ClimbRamp --- //

        // --- Begin InchForwards --- //
        case StepState_Ramp::InchForwards:
        setServoTarget(&baseServo, 160);
        updateServo(&baseServo);
        runPID_withHysteresis(1400);
        if (millis() - timeCheckpoint > INCH_FORWARDS_TIME) {
          currentPetState = PetState::Pet2;
          drivetrainSetPower(0);
          timeCheckpoint = millis();
          delay(1000);
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
        runPID_withHysteresis(1000);
        if (timeOfFlightReading < 65) {
          currentStepState_Pet2 = StepState_Pet2::ArmSearchPreset2;
          drivetrainSetPower(0);
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
          baseServo.speed = 0.02;
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
        height = 12.0;
        nextShoulderAngle = shoulderServo.currentAngle - 1;
        nextElbowAngle = calculateElbowAngle(nextShoulderAngle, height);
        nextWristAngle = calculateWristAngle(nextShoulderAngle, nextElbowAngle, height);
        setAllServoTargets(baseServo.currentAngle, nextShoulderAngle, nextElbowAngle, nextWristAngle, clawServo.currentAngle);
        while (!allServosDone()) {
          updateServos();
        }
        if (timeOfFlightReading < TOF_DETECTION_THRESHOLD || shoulderServo.targetAngle < 5) {
          servoGoTo(&clawServo, CLAW_CLOSED_ANGLE);
          currentStepState_Pet2 = StepState_Pet2::ReturnArm2;
          delay(CLAW_CLOSE_TIME);
          timeCheckpoint = millis();
        }
        break;
        // --- End PetGrab2 --- //

        // --- Begin ReturnArm2 --- //
        case StepState_Pet2::ReturnArm2:
        setAllServoTargets(90, 90, 160, 127, CLAW_CLOSED_ANGLE);
        while (!allServosDone()) {
          updateServos();
        } 
        if (allServosDone()) {
          currentStepState_Pet2 = StepState_Pet2::DropPet2;
          setServoTarget(&baseServo, 160);
          while (!servoDone(&baseServo)) {
            updateServo(&baseServo);
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
        }
        servoGoTo(&clawServo, 130);
        delay(CLAW_CLOSE_TIME);
        setAllServoTargets(120, 70, 170, 160, 130);
        while (!allServosDone()) {
          updateServos();
        }
        currentPetState = PetState::Pet3;
        setPIDValues(1.2, 0.0, 0.0, 0.0);
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
        runPID_withBackup(700);
        if (timeOfFlightReading < 120) {
          currentStepState_Pet3 = StepState_Pet3::ArmSearchPreset3;
          drivetrainSetPower(0);
          timeCheckpoint = millis();
        }
        break;
        // --- End FindTarget3 --- //

        // --- Begin ArmSearchPreset3 --- //
        case StepState_Pet3::ArmSearchPreset3:
        setServoTarget(&baseServo, PET3_UPPER_SWEEP_ANGLE);
        updateServo(&baseServo);
        if (servoDone(&baseServo)) {
          currentStepState_Pet3 = StepState_Pet3::PetSearch3;
          baseServo.speed = 0.02;
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
          timeOfFlightArray[sweepAngleIncrement] = (double) timeOfFlightSum / timeOfFlightCount;
          timeOfFlightSum = 0;
          timeOfFlightCount = 0;
          sweepAngleIncrement++;
          servoGoTo(&baseServo, PET3_LOWER_SWEEP_ANGLE + sweepAngleIncrement);
          if (sweepAngleIncrement == PET3_UPPER_SWEEP_ANGLE - PET3_LOWER_SWEEP_ANGLE) {
            arrayValueMark = timeOfFlightArray[0];
            for (int i = 0; arrayValueMark > TOF_RANGE_THRESHOLD && i < PET3_UPPER_SWEEP_ANGLE - PET3_LOWER_SWEEP_ANGLE; i++) {
              if (timeOfFlightArray[i] < TOF_RANGE_THRESHOLD) {
                arrayValueMark = timeOfFlightArray[i];
                lowerAngleMark = i;
              }
            }
            arrayValueMark = timeOfFlightArray[PET3_UPPER_SWEEP_ANGLE - PET3_LOWER_SWEEP_ANGLE - 1];
            for (int i = PET3_UPPER_SWEEP_ANGLE - PET3_LOWER_SWEEP_ANGLE - 1; arrayValueMark > TOF_RANGE_THRESHOLD && i >= 0; i--) {
              if (timeOfFlightArray[i] < TOF_RANGE_THRESHOLD) {
                arrayValueMark = timeOfFlightArray[i];
                upperAngleMark = i;
              }
            }
            maxTOFbaseAngle = PET3_LOWER_SWEEP_ANGLE + (int) ((lowerAngleMark + upperAngleMark) / 2.0);
          }
          currentStepState_Pet3 = StepState_Pet3::PetFound3;
        }
        break;
        // --- End PetSearch3 --- //

        // --- Begin PetFound3 --- //
        case StepState_Pet3::PetFound3:
        setServoTarget(&baseServo, maxTOFbaseAngle);
        while (!servoDone(&baseServo)) {
          updateServo(&baseServo);
        }
        delay(100000);
        break;
        // --- End PetFound3 --- //

        // --- Begin PetGrab3 --- //
        case StepState_Pet3::PetGrab3:
        break;
        // --- End PetGrab3 --- //

        // --- Begin DropPet3 --- //
        case StepState_Pet3::DropPet3:
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
        break;
        // --- End FindTarget4 --- //

        // --- Begin ArmSearchPreset4 --- //
        case StepState_Pet4::ArmSearchPreset4:
        break;
        // --- End ArmSearchPreset4 --- //

        // --- Begin PetSearch4 --- //
        case StepState_Pet4::PetSearch4:
        break;
        // --- End PetSearch4 --- //

        // --- Begin PetFound4 --- //
        case StepState_Pet4::PetFound4:
        break;
        // --- End PetFound4 --- //

        // --- Begin PetGrab4 --- //
        case StepState_Pet4::PetGrab4:
        break;
        // --- End PetGrab4 --- //

        // --- Begin DropPet4 --- //
        case StepState_Pet4::DropPet4:
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
        break;
        // --- End FindTarget5 --- //

        // --- Begin ArmSearchPreset5 --- //
        case StepState_Pet5::ArmSearchPreset5:
        break;
        // --- End ArmSearchPreset5 --- //

        // --- Begin PetSearch5 --- //
        case StepState_Pet5::PetSearch5:
        break;
        // --- End PetSearch5 --- //

        // --- Begin PetFound5 --- //
        case StepState_Pet5::PetFound5:
        break;
        // --- End PetFound5 --- //

        // --- Begin PetGrab5 --- //
        case StepState_Pet5::PetGrab5:
        break;
        // --- End PetGrab5 --- //

        // --- Begin DropPet5 --- //
        case StepState_Pet5::DropPet5:
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
        break;
        // --- End FindTarget6 --- //

        // --- Begin ArmSearchPreset6 --- //
        case StepState_Pet6::ArmSearchPreset6:
        break;
        // --- End ArmSearchPreset6 --- //

        // --- Begin PetSearch6 --- //
        case StepState_Pet6::PetSearch6:
        break;
        // --- End PetSearch6 --- //

        // --- Begin PetFound6 --- //
        case StepState_Pet6::PetFound6:
        break;
        // --- End PetFound6 --- //

        // --- Begin PetGrab6 --- //
        case StepState_Pet6::PetGrab6:
        break;
        // --- End PetGrab6 --- //

        // --- Begin DropPet6 --- //
        case StepState_Pet6::DropPet6:
        break;
        // --- End DropPet6 --- //

      }
      break;
      // --- End Pet6 --- //

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
  delay(250);
}

void updateSwitchState() {
    // Read switch states
    initializeSwitchState = !((bool) digitalRead(INITIALIZE_SWITCH_PIN));
    resetSwitchState = !((bool) digitalRead(RESET_SWITCH_PIN));

    // Compute states
    if (initializeSwitchState && resetSwitchState) {
      currentSwitchState = SwitchState::Run;
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
    if (leftOnTape || rightOnTape) {
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
