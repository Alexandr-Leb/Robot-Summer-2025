// --- Header Files --- //
// Header Files - External
#include <Arduino.h>

// Header Files - Internal
#include "drivetrain.h"
#include "sensors.h"

// --- Constants --- //
const double HYSTERESIS_MULTIPLIER = 25;

// --- Variables --- //
// Variables - PID
extern double k_p = 1.5;
extern double k_i = 0.0; // Not using
extern double k_d = 2.0;
extern double pValue;
extern double iValue; // Not using
extern double dValue;
extern int error;
extern int prevError;
extern long prevTime; // in us
extern double k_e = 0.0;
extern double eValue;

// --- Function Headers --- //
void pidSetup();
void computePID();
void runPID(int power);

// --- Functions --- //
void pidSetup() {
  // Variables Setup
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

void runPID(int power) {
  readReflectanceSensors();
  computePID();
  leftMotorSetPower((int) (power + pValue + dValue - abs(eValue)));
  rightMotorSetPower((int) (power - pValue - dValue - abs(eValue)));
}

void runHysteresis(int power) {
  leftMotorSetPower(power + (int) (prevError * HYSTERESIS_MULTIPLIER));
  rightMotorSetPower(power - (int) (prevError * HYSTERESIS_MULTIPLIER));
}
