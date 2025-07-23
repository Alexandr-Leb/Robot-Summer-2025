// LIS3MDL	 ---  ESP32 Pico
// VIN	3.3V
// GND	             GND
// SDA	             GPIO 21 (Default)
// SCL	             GPIO 22 (Default)

#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_LIS3MDL lis;

float previousMagnitude = 0;
const float threshold = 1.0;  // Minimum change to consider significant (uT)

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  if (!lis.begin_I2C()) {
    Serial.println("Failed to find LIS3MDL chip");
    while (1) delay(10);
  }

  Serial.println("LIS3MDL found!");
  lis.setRange(LIS3MDL_RANGE_4_GAUSS);
}

void loop() {
  sensors_event_t event;
  lis.getEvent(&event);

  float x = event.magnetic.x;
  float y = event.magnetic.y;
  float z = event.magnetic.z;

  float magnitude = sqrt(x * x + y * y + z * z);

  Serial.print("Magnetic Field Magnitude: ");
  Serial.print(magnitude);
  Serial.print(" uT");

  // Compare to previous
  float delta = magnitude - previousMagnitude;
  // Serial.print(" | Delta: ");
  // Serial.print(delta);

  Serial.print(" | X: ");
  Serial.print(x);
  Serial.print(" | Y: ");
  Serial.print(y);
  Serial.print(" | Z: ");
  Serial.print(z);

  Serial.println();


  previousMagnitude = magnitude;
  delay(500);
}


