// Compressions_SensorHub.ino

#include "SensorHub_Settings.h"
#include "PinManager.h"
#include "wireHelper.h"
#include "adcHelper.h"
#include "ledHelper.h"
#include "DeviceManager.h"
#include "serialHelper.h"

// Pin Definitions
#define ledHb 14

// i2c system bus - Define these here or in SensorHub_Settings.h
#define W0_SCL 27  // PA22
#define W0_SDA 26  // PA23

#define W1_SCL 39  // PA13
#define W1_SDA 28  // PA12

#define W2_SCL 13  // PA17
#define W2_SDA 11  // PA16   

// Create Wire instances
TwoWire WireSensorA(&sercom1, W1_SDA, W1_SCL);  // Sensor Port A
TwoWire WireSensorB(&sercom4, W2_SDA, W2_SCL);  // Sensor Port B

// Create helper class instances
WireHelper wireHelperA(WireSensorA, "A");
WireHelper wireHelperB(WireSensorB, "B");
AdcHelper adcHelper(WireSensorA, ADS7138_Address);
LedHelper ledHelper(ledHb);

// Dependency Injection for DeviceManager
DeviceManager deviceManager(wireHelperA, adcHelper, ADS7138_Address);

void setup() {
  setupSerial();
  waitForSerial();
  wireHelperA.begin();
  wireHelperB.begin();

  PinManager::setupI2CPins(); // Call the pin configuration method


  ledHelper.off();  // Turn off heartbeat LED initially
  deviceManager.discoverAndInit();
}

void loop() {
  ledHelper.update();
  deviceManager.readSensorData();
}