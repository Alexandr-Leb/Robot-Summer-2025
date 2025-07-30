// --- Header Files --- //
// Header Files - External
#include <Arduino.h>
#include "driver/adc.h"
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

// Header Files - Internal
#include "esp_constants.h"

// --- Constants --- //
// Constants - Reflectance Sensor
const int REFLECTANCE_THRESHOLD_OFFSET = 400;

// --- Variables --- //
// Variables - Reflectance Sensor
extern int leftReflectance;
extern int rightReflectance;

// Variables - Reflectance Sensor Initialization
extern int leftReflectanceThreshold;
extern int rightReflectanceThreshold;
extern int leftReflectanceThresholdSum;
extern int rightReflectanceThresholdSum;
extern int reflectanceAverageLoopCounter;

// Variables - Magnetometer
extern Adafruit_LIS3MDL lis;
extern double magnetometerMagnitude;

// Variables - Magnetometer Average Calculations
extern double magnetometerMagnitudeSum;
extern int magnetometerAverageCount;

// --- Function Headers --- //
void sensorSetup();
void reflectanceSetup();
void magnetometerSetup();
void readReflectanceSensors();
void readMagnetometer();
void initializeReflectanceSensors();

// --- Functions --- //
void sensorSetup() {
  reflectanceSetup();
  magnetometerSetup();
}

void reflectanceSetup() {
  // Variable Initialization
  leftReflectance = 0;
  rightReflectance = 0;
  leftReflectanceThresholdSum = 0;
  rightReflectanceThresholdSum = 0;
  reflectanceAverageLoopCounter = 0;

  // Analog Input Setup
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(LEFT_REFLECTANCE_PIN, ADC_ATTEN_DB_12);
  adc1_config_channel_atten(RIGHT_REFLECTANCE_PIN, ADC_ATTEN_DB_12);
}

void magnetometerSetup() {
  // Variables Initialization
  magnetometerMagnitude = 0.0;
  magnetometerMagnitudeSum = 0.0;
  magnetometerAverageCount = 0;

  // I2C Setup
  lis.begin_I2C();
}

void readReflectanceSensors() {
  leftReflectance = adc1_get_raw(LEFT_REFLECTANCE_PIN);
  rightReflectance = adc1_get_raw(RIGHT_REFLECTANCE_PIN);
}

void readMagnetometer() {
  sensors_event_t event;
  lis.getEvent(&event);
  float x = event.magnetic.x;
  float y = event.magnetic.y;
  float z = event.magnetic.z;
  magnetometerMagnitude = (double) sqrt(x * x + y * y + z * z);
}

void initializeReflectanceSensors(int initializeTime) {
  int startTime = millis();
  while (millis() - startTime < initializeTime) {
    leftReflectanceThresholdSum += adc1_get_raw(LEFT_REFLECTANCE_PIN);
    rightReflectanceThresholdSum += adc1_get_raw(RIGHT_REFLECTANCE_PIN);
    reflectanceAverageLoopCounter++;
  }
  leftReflectanceThreshold = (int) ((double) leftReflectanceThresholdSum / reflectanceAverageLoopCounter) + REFLECTANCE_THRESHOLD_OFFSET;
  rightReflectanceThreshold = (int) ((double) rightReflectanceThresholdSum / reflectanceAverageLoopCounter) + REFLECTANCE_THRESHOLD_OFFSET;
}
