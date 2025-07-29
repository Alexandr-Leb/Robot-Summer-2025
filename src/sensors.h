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
void sensorSetup() {
  // Reflectance Sensor Analog Input Setup
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(LEFT_REFLECTANCE_PIN, ADC_ATTEN_DB_12);
  adc1_config_channel_atten(RIGHT_REFLECTANCE_PIN, ADC_ATTEN_DB_12);

  // Reflectance Sensor Variables Setup

  // Magnetometer Sensor I2C Setup
  lis.begin_I2C();

  // Magnetometer Sensor Setup
}

void initializeReflectanceSensors() {}

void readReflectanceSensors() {}

void readMagnetometer() {}
