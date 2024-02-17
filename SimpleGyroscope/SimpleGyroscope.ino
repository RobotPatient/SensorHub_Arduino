/*
  Arduino BMI270 - Simple Gyroscope

  This example reads the gyroscope values from the BMI270
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense Rev2

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include <Wire.h>            // SERCOM3?
#include "wiring_private.h"  // pinPeripheral() function
#include "pinTools.h"
#include "Arduino_BMI270_BMM150.h"

#define W2_SCL 13  // PA17
#define W2_SDA 11  // PA16

// i2c system bus
#define W0_SCL 27  // PA22
#define W0_SDA 26  // PA23

#define W1_SCL 39  // PA13
#define W1_SDA 28  // PA12

#define ledHb 14

TwoWire WireBackbone(&sercom3, W0_SDA, W0_SCL);  // Main
TwoWire WireSensorA(&sercom1, W1_SDA, W1_SCL);   // Sensor A
TwoWire WireSensorB(&sercom4, W2_SDA, W2_SCL);   // Sensor B

TwiPinPair portBackbone = TwiPinPair(W0_SCL, W0_SDA);
TwiPinPair portSensorsA = TwiPinPair(W1_SCL, W1_SDA);
TwiPinPair portSensorsB = TwiPinPair(W2_SCL, W2_SDA);

BoschSensorClass imu = BoschSensorClass(WireSensorB);

void initWire() {
  Wire.begin();
  WireBackbone.begin();
  WireSensorA.begin();
  WireSensorB.begin(); // dat doen de malloten in de library zelf al.

  portBackbone.setPinPeripheralStates();
  portSensorsA.setPinPeripheralAltStates();
  portSensorsB.setPinPeripheralStates();
}

void setup() {
  Serial.begin(115200);
  
  while (!Serial)
    ;
  Serial.println("Started");

  initWire();

  Serial.println("Wires initialized...");


  if (!imu.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(imu.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
}

void loop() {
  float x, y, z;

  if (imu.gyroscopeAvailable()) {
    imu.readGyroscope(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }
}
