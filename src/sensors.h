// --- Header Files --- //
// Header Files - External
#include <Arduino.h>
#include "driver/adc.h"
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

// Header Files - Internal
#include "esp_constants.h"

// --- Variables --- //
// Variables - Reflectance Sensor
extern int leftReflectance;
extern int rightReflectance;
extern int leftReflectanceThresholdSum;
extern int rightReflectanceThresholdSum;
extern int reflectanceAverageLoopCounter;

// Variables - Magnetometer
extern Adafruit_LIS3MDL lis;
extern double magnetometerMagnitude;
extern double magnetometerMagnitudeSum;
extern int magnetometerAverageCount;

// --- Functions --- //
void sensorSetup() {}

void initializeReflectanceSensors() {}

void readReflectanceSensors() {}

void readMagnetometer() {}
