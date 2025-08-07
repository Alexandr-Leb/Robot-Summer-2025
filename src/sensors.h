// --- Header Files --- //
// Header Files - External
#include <Arduino.h>
#include "driver/adc.h"
#include <Wire.h>
#include <Adafruit_VL53L1X.h> // Time of flight
#include <Adafruit_LIS3MDL.h> // Magnetometer
#include <Adafruit_Sensor.h>

// Header Files - Internal
#include "esp_constants.h"

#define XSHUT_PIN  -1
#define IRQ_PIN    -1

// --- Constants --- //
// Constants - Reflectance Sensor
const int REFLECTANCE_THRESHOLD_OFFSET = -200;

// --- Variables --- //
// Variables - Reflectance Sensor
extern int leftReflectance;
extern int rightReflectance;
extern int forwardLeftReflectance;
extern int forwardRightReflectance;

// Variables - Reflectance Sensor Initialization
extern int leftReflectanceThreshold;
extern int leftReflectanceThresholdSum;
extern int rightReflectanceThreshold;
extern int rightReflectanceThresholdSum;
extern int forwardLeftReflectanceThreshold;
extern int forwardLeftReflectanceThresholdSum;
extern int forwardRightReflectanceThreshold;
extern int forwardRightReflectanceThresholdSum;
extern int reflectanceAverageLoopCounter;

// Variables - Magnetometer
extern Adafruit_LIS3MDL lis;
extern double magnetometerMagnitude;

// Variables - Time of Flight
extern Adafruit_VL53L1X tof;
extern SemaphoreHandle_t i2cMutex;
extern volatile uint16_t timeOfFlightReading;

// Variables - Hysteresis
extern bool leftOnTape;
extern bool rightOnTape;
extern bool prevLeftOnTape;
extern bool prevRightOnTape;

// --- Function Headers --- //
void sensorSetup();
void reflectanceSetup();
void magnetometerSetup();
void timeOfFlightSetup();
void readReflectanceSensors();
void readForwardReflectanceSensors();
void readMagnetometer();
// void readTimeOfFlight();
void initializeReflectanceSensors(int initializeTime);
void ToFTask(void *pv);

// --- Functions --- //
void sensorSetup() {
  Wire.begin();
  reflectanceSetup();
  magnetometerSetup();
  timeOfFlightSetup();
}

void reflectanceSetup() {
  // Analog Input Setup
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(LEFT_REFLECTANCE_PIN, ADC_ATTEN_DB_12);
  adc1_config_channel_atten(RIGHT_REFLECTANCE_PIN, ADC_ATTEN_DB_12);
  adc1_config_channel_atten(FORWARD_LEFT_REFLECTANCE_PIN, ADC_ATTEN_DB_12);
  adc1_config_channel_atten(FORWARD_RIGHT_REFLECTANCE_PIN, ADC_ATTEN_DB_12);

  // Variable Initialization
  leftReflectance = 0;
  rightReflectance = 0;
  forwardLeftReflectance = 0;
  forwardRightReflectance = 0;
  leftReflectanceThresholdSum = 0;
  rightReflectanceThresholdSum = 0;
  forwardLeftReflectanceThresholdSum = 0;
  forwardRightReflectanceThresholdSum = 0;
  reflectanceAverageLoopCounter = 0;
}

void magnetometerSetup() {
  // Sensor Setup
  lis.begin_I2C();

  // Variables Initialization
  magnetometerMagnitude = 0.0;
}

void timeOfFlightSetup() {
  // Sensor Setup
  i2cMutex = xSemaphoreCreateMutex();          // create before tasks

  /* Kick the task onto core-0 (0 = PRO_CPU, 1 = APP_CPU)  */
  xTaskCreatePinnedToCore(ToFTask, "VL53L1_reader", 4096, nullptr, 1, nullptr, 0);
}

void readReflectanceSensors() {
  leftReflectance = adc1_get_raw(LEFT_REFLECTANCE_PIN);
  rightReflectance = adc1_get_raw(RIGHT_REFLECTANCE_PIN);

  // Hysteresis Computation
  prevLeftOnTape = leftOnTape;
  prevRightOnTape = rightOnTape;
  leftOnTape = (leftReflectance > leftReflectanceThreshold);
  rightOnTape = (rightReflectance > rightReflectanceThreshold);
}

void readForwardReflectanceSensors() {
  forwardLeftReflectance = adc1_get_raw(FORWARD_LEFT_REFLECTANCE_PIN);
  forwardRightReflectance = adc1_get_raw(FORWARD_RIGHT_REFLECTANCE_PIN);
}

void readMagnetometer() {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  sensors_event_t event;
  lis.getEvent(&event);
  float x = event.magnetic.x;
  float y = event.magnetic.y;
  float z = event.magnetic.z;
  magnetometerMagnitude = (double) sqrt(x * x + y * y + z * z);
  xSemaphoreGive(i2cMutex);
}

// void readTimeOfFlight() {
//   tof.readRange();
// }

void initializeReflectanceSensors(int initializeTime) {
  int startTime = millis();
  while (millis() - startTime < initializeTime) {
    leftReflectanceThresholdSum += adc1_get_raw(LEFT_REFLECTANCE_PIN);
    rightReflectanceThresholdSum += adc1_get_raw(RIGHT_REFLECTANCE_PIN);
    forwardLeftReflectanceThresholdSum += adc1_get_raw(FORWARD_LEFT_REFLECTANCE_PIN);
    forwardRightReflectanceThresholdSum += adc1_get_raw(FORWARD_RIGHT_REFLECTANCE_PIN);
    reflectanceAverageLoopCounter++;
  }
  leftReflectanceThreshold = (int) ((double) leftReflectanceThresholdSum / reflectanceAverageLoopCounter) + REFLECTANCE_THRESHOLD_OFFSET;
  rightReflectanceThreshold = (int) ((double) rightReflectanceThresholdSum / reflectanceAverageLoopCounter) + REFLECTANCE_THRESHOLD_OFFSET;
  forwardLeftReflectanceThreshold = (int) ((double) forwardLeftReflectanceThresholdSum / reflectanceAverageLoopCounter) + REFLECTANCE_THRESHOLD_OFFSET;
  forwardRightReflectanceThreshold = (int) ((double) forwardRightReflectanceThresholdSum / reflectanceAverageLoopCounter) + REFLECTANCE_THRESHOLD_OFFSET;
}

void ToFTask(void *pv) {
  if (!tof.begin(0x29, &Wire)) {              // default addr 0x29
    Serial.println("VL53L1X NOT FOUND – task ends");
    vTaskDelete(NULL);
  }
  tof.setTimingBudget(50);                    // 15-500 ms valid
  tof.startRanging();                         // continuous mode

  const TickType_t period = pdMS_TO_TICKS(40); // ≈25 Hz
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    if (tof.dataReady()) {
      int16_t d = tof.distance();             // −1 = invalid / out-of-range
      if (d != -1)      timeOfFlightReading = (uint16_t)d;
      else              timeOfFlightReading = 65535U;   // sentinel
      tof.clearInterrupt();
    }
    xSemaphoreGive(i2cMutex);
    vTaskDelayUntil(&lastWake, period);
  }
}
